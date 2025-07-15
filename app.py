import os
import re
import uuid
import time
import logging
import asyncio 
import shutil
import subprocess
import tempfile
from fastapi import FastAPI, UploadFile, HTTPException, Request, Form
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import signal
import math
from typing import Union, Optional, List 
from pathlib import Path 
import sqlite3 
import struct  
import httpx

# Set logging to DEBUG to capture detailed logs
logging.basicConfig(format="%(asctime)s [%(levelname)s] %(message)s", level=logging.DEBUG)
logger = logging.getLogger("rtabmap")
logger.setLevel(logging.DEBUG)

app = FastAPI()

@app.on_event("startup")
async def startup_event():
    """
    Initializes the global RTAB-Map service instance when the application starts.
    """
    global rtabmap_service
    rtabmap_service = RTABMapService()
    logger.info("RTAB-Map service created on application startup.")

@app.on_event("shutdown")
async def shutdown_event():
    """
    Properly shuts down the RTAB-Map service when the application is closing.
    """
    if rtabmap_service:
        await rtabmap_service.shutdown()
        logger.info("RTAB-Map service shut down successfully.")

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Define base data and upload directories
DATA_DIR = Path(os.getenv("DATA_DIR", "/data"))
UPLOAD_BASE_DIR = DATA_DIR / "uploads"

# --- Wayfinder Integration ---
WAYFINDER_URL = "https://jennet-crisp-molly.ngrok-free.app/wayfinder"

# --- Coordinate System Transformation ---

# 1. Define reference points in RTAB-Map
# These are the known real-world coordinates within the RTAB-Map's coordinate system.
PROF_ROOM_SYS1 = {"x": 5.088, "y": 1.850}
LAB_ROOM_SYS1 = {"x": 0.227, "y": 13.160}

# 2. Define corresponding reference points in Wayfinder system
# These are the coordinates in the target system that match the points from RTAB-Map.
PROF_ROOM_SYS2 = {"x": 73.60, "y": 23.71}
LAB_ROOM_SYS2 = {"x": 62.59, "y": 15.14}

# 3. Calculate the transformation parameters (rotation and translation)
# This is done once on startup to be used for all subsequent transformations.

# Calculate the vector between the two reference points in RTAB-Map
v1 = (LAB_ROOM_SYS1["x"] - PROF_ROOM_SYS1["x"], LAB_ROOM_SYS1["y"] - PROF_ROOM_SYS1["y"])
# Calculate the vector between the two reference points in Wayfinder system
v2 = (LAB_ROOM_SYS2["x"] - PROF_ROOM_SYS2["x"], LAB_ROOM_SYS2["y"] - PROF_ROOM_SYS2["y"])

# Calculate the rotation angle (theta) required to align v1 with v2.
# The angle is the difference between the angles of the two vectors.
theta = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])

# Pre-calculate the cosine and sine of the rotation angle for efficiency.
cos_theta = math.cos(theta)
sin_theta = math.sin(theta)

# Calculate the translation (shift) needed after rotation.
# 1. Rotate a point from RTAB-Map.
prof_room_sys1_rotated_x = PROF_ROOM_SYS1["x"] * cos_theta - PROF_ROOM_SYS1["y"] * sin_theta
prof_room_sys1_rotated_y = PROF_ROOM_SYS1["x"] * sin_theta + PROF_ROOM_SYS1["y"] * cos_theta

# 2. Find the difference between the rotated RTAB-Map point and the target Wayfinder system point.
shift_x = PROF_ROOM_SYS2["x"] - prof_room_sys1_rotated_x
shift_y = PROF_ROOM_SYS2["y"] - prof_room_sys1_rotated_y

def transform_coordinates(x1: float, y1: float) -> dict:
    """
    Transforms coordinates from RTAB-Map to Wayfinder system.
    Applies a calculated rotation and translation.
    """
    # Step 1: Apply the rotation to the input coordinates.
    x_rotated = x1 * cos_theta - y1 * sin_theta
    y_rotated = x1 * sin_theta + y1 * cos_theta
    
    # Step 2: Apply the translation to the rotated coordinates.
    x2 = x_rotated + shift_x
    y2 = y_rotated + shift_y
    
    return {"x": round(x2, 2), "y": round(y2, 2)}

# --- End of Coordinate System Transformation ---

async def send_to_wayfinder(x: float, y: float):
    """
    Sends the user's current X and Y coordinates to the wayfinder API.
    """
    payload = {
        "action": "update",
        "currentX": x,
        "currentY": y
    }
    try:
        async with httpx.AsyncClient() as client:
            logger.info(f"Sending to wayfinder: {payload}")
            response = await client.post(WAYFINDER_URL, json=payload)
            response.raise_for_status()  # Raise an exception for non-2xx status codes
            logger.info(f"Successfully sent coordinates to wayfinder. Response: {response.json()}")
            return response.json()
    except httpx.RequestError as e:
        logger.error(f"Error sending coordinates to wayfinder: {e}")
        return {"error": str(e)}

# Global variable to hold the RTABMap service
rtabmap_service = None

# --- Utility Functions ---

def quaternion_to_rpy(qx, qy, qz, qw):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    Returns angles in radians.
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def rotation_matrix_to_quaternion(r11, r12, r13, r21, r22, r23, r31, r32, r33):
    """
    Convert a 3x3 rotation matrix to a quaternion (qx, qy, qz, qw).
    """
    tr = r11 + r22 + r33
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (r32 - r23) / S
        qy = (r13 - r31) / S
        qz = (r21 - r12) / S
    elif (r11 > r22) and (r11 > r33):
        S = math.sqrt(1.0 + r11 - r22 - r33) * 2
        qw = (r32 - r23) / S
        qx = 0.25 * S
        qy = (r12 + r21) / S
        qz = (r13 + r31) / S
    elif r22 > r33:
        S = math.sqrt(1.0 + r22 - r11 - r33) * 2
        qw = (r13 - r31) / S
        qx = (r12 + r21) / S
        qy = 0.25 * S
        qz = (r23 + r32) / S
    else:
        S = math.sqrt(1.0 + r33 - r11 - r22) * 2
        qw = (r21 - r12) / S
        qx = (r13 + r31) / S
        qy = (r23 + r32) / S
        qz = 0.25 * S
    return qx, qy, qz, qw

class RTABMapService:
    """
    A persistent RTAB-Map service that keeps the database loaded in memory.
    Uses a long-running RTABMap process and communicates with it via temporary files.
    """
    def __init__(self):
        self.db_path: Optional[Path] = None
        self.is_initialized = False
        self.lock = asyncio.Lock()
        self.image_counter = 0
        self.db_connection_cache = {}   # Cache for database connections
        self.node_poses_cache = {}      # Cache for preloaded node poses
        self.base_rtabmap_params: List[str] = [] # Base parameters for rtabmap-console

    def _parse_and_format_pose(self, node_id, pose_data, precision=5):
        """Helper to parse pose data from DB (blob or string) and format it."""
        try:
            if isinstance(pose_data, bytes):
                if len(pose_data) == 12 * 4:  # 12 floats
                    values = struct.unpack('12f', pose_data)
                elif len(pose_data) == 12 * 8:  # 12 doubles
                    values = struct.unpack('12d', pose_data)
                else:
                    logger.warning(f"Skipping node {node_id}: Pose data BLOB has unexpected length: {len(pose_data)}")
                    return None
            elif isinstance(pose_data, str):
                values = [float(x) for x in pose_data.split()]
                if len(values) != 12:
                    logger.warning(f"Skipping node {node_id}: Pose data string has wrong number of values: {len(values)}")
                    return None
            else:
                logger.warning(f"Skipping node {node_id}: Unknown pose data type: {type(pose_data)}")
                return None

            r11, r21, r31 = values[0], values[1], values[2]
            r12, r22, r32 = values[3], values[4], values[5]
            r13, r23, r33 = values[6], values[7], values[8]
            tx, ty, tz = values[9], values[10], values[11]
            
            # Convert rotation matrix to quaternion, then to Euler angles (roll, pitch, yaw)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(r11, r12, r13, r21, r22, r23, r31, r32, r33)
            roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
            
            # Return the formatted pose dictionary
            return {
                "x": round(tx, precision),
                "y": round(ty, precision),
                "z": round(tz, precision), 
                "roll": round(roll, precision), "pitch": round(pitch, precision), "yaw": round(yaw, precision), 
                "map_id": node_id
            }
        except Exception as e:
            logger.error(f"Error processing pose for node {node_id}: {e}")
            return None

    async def initialize(self, db_path_obj: Path):
        async with self.lock:
            # If service is already initialized with the same DB, no need to reinitialize
            if self.is_initialized and self.db_path == db_path_obj:
                logger.info(f"RTAB-Map service already initialized with database: {db_path_obj}")
                return True

            # Shut down any existing service before initializing a new one
            await self.shutdown() # Resets flags

            logger.info(f"Initializing RTAB-Map service with database: {db_path_obj}")
            if not db_path_obj.is_file():
                logger.error(f"Database file not found: {db_path_obj}")
                return False
            self.db_path = db_path_obj

            # Define the set of parameters for rtabmap-console for localization.
            # These are optimized for performance and accuracy in a localization-only scenario.
            self.base_rtabmap_params = [
                "--Rtabmap/LoadDatabaseParameters", "true",  # Load parameters from the database
                "--Mem/IncrementalMemory", "false",         # Disable incremental memory (static map)
                "--Mem/InitWMWithAllNodes", "false",        # Do not initialize working memory with all nodes
                "--Kp/DetectorStrategy", "6",               # Use ORB feature detector
                "--Vis/FeatureType", "6",                   # Use ORB features for visual matching
                "--BRIEF/Bytes", "64",                      # Set BRIEF descriptor size to 64 bytes
                "--Kp/MaxFeatures", "800",                  # Maximum number of features per image
                "--Rtabmap/LoopThr", "0.06",                # Loop closure threshold
                "--Optimizer/Strategy", "1",                 # Use GTSAM optimizer
                "--Rtabmap/DetectionRate", "0",              # Detection rate (0 = as fast as possible)
                "--RGBD/ProximityPathMaxNeighbors", "1",     # Max neighbors for proximity path
                "--RGBD/NeighborLinkRefining", "true",       # Refine neighbor links
                "--Vis/MinInliers", "15",                    # Minimum inliers for visual match
                "--Vis/PnPFlags", "1",                       # Use PnP RANSAC for pose estimation
                "--Rtabmap/PublishLastLocalizationPose", "false", # Do not publish last localization pose
                "--Rtabmap/StatisticLogged", "true",         # Enable logging of statistics
                "--Mem/ImagePreDecimation", "4",             # Downsample images by factor of 4
                "--Mem/LaserScanDownsampleStep", "2",        # Downsample laser scans by step of 2
                "--Mem/NotLinkedNodesKept", "false",         # Do not keep not-linked nodes
                "--Rtabmap/ImagesAlreadyRectified", "true",  # Assume images are already rectified
                "--logconsole",
                "--uinfo"                                      # Set log level to info
            ]

            # The service is now considered "initialized" and ready for per-image processing.
            logger.info("RTAB-Map service will operate by launching rtabmap-console per image.")
            self.is_initialized = True
            await self.preload_node_poses() # Preload poses for efficiency
            return True

    async def get_stored_node_pose(self, node_id: int) -> Union[dict, None]:
        """
        Retrieves the stored pose directly from the SQLite database, using caches.
        """
        if not self.db_path:
            return None
            
        # Return from cache if available for speed
        if node_id in self.node_poses_cache:
            return self.node_poses_cache[node_id]
        
        # Get or create a database connection from the cache
        db_path_str = str(self.db_path)
        conn = self.db_connection_cache.get(db_path_str)
        if not conn:
            try:
                conn = sqlite3.connect(db_path_str)
                self.db_connection_cache[db_path_str] = conn
            except Exception as e:
                logger.error(f"Failed to connect to database {db_path_str}: {e}")
                return None

        try:
            cursor = conn.cursor()
            # Fetch the pose data for the given node ID
            cursor.execute("SELECT pose FROM Node WHERE id = ?", (node_id,))
            result = cursor.fetchone()
            
            if result:
                # Parse the raw pose data into a structured dictionary
                parsed_pose = self._parse_and_format_pose(node_id, result[0])
                if parsed_pose:
                    self.node_poses_cache[node_id] = parsed_pose # Cache the result
                return parsed_pose
            else:
                logger.warning(f"Node {node_id} not found in 'Node' table")
                return None
        except Exception as e:
            logger.error(f"Error querying database for node {node_id}: {e}")
            return None

    async def process_image(self, image_path: Path):
        if not self.is_initialized or not self.db_path:
            raise RuntimeError("RTAB-Map service not initialized or DB path not set.")

        async with self.lock:
            # Create a unique, temporary directory for this processing request
            self.image_counter += 1
            request_id = f"req_{self.image_counter}_{int(time.time())}"
            image_processing_dir = UPLOAD_BASE_DIR / f"img_proc_{request_id}"
            image_name = image_path.name
            start_time_total = time.perf_counter()
            
            try:
                # Prepare the processing directory
                image_processing_dir.mkdir(parents=True, exist_ok=True)
                target_image_in_processing_dir = image_processing_dir / image_name
                shutil.copy(image_path, target_image_in_processing_dir)

                # Run localization five times and aggregate results
                results = []
                for _ in range(5):
                    per_image_cmd = ["rtabmap-console", "-input", str(self.db_path)] + self.base_rtabmap_params + [str(image_processing_dir)]
                    proc_img_loc = await asyncio.create_subprocess_exec(*per_image_cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    stdout_bytes, stderr_bytes = await asyncio.wait_for(proc_img_loc.communicate(), timeout=120.0)
                    output_text_localization = stdout_bytes.decode(errors='ignore') + stderr_bytes.decode(errors='ignore')
                    if proc_img_loc.returncode != 0:
                        logger.error(f"RTAB-Map per-image launch failed for {image_name}. RC={proc_img_loc.returncode}")
                    result = parse_localization_output(output_text_localization)
                    if result:
                        results.append(result)

                # Aggregate results using a voting mechanism
                pic_id_votes = {}
                for result in results:
                    pic_id = result.get("pic_id")
                    if pic_id is not None:
                        pic_id_votes[pic_id] = pic_id_votes.get(pic_id, 0) + 1

                # Determine the most consistent match
                if pic_id_votes:
                    most_voted_pic_id = max(pic_id_votes, key=pic_id_votes.get)
                    final_pose_data = next((res for res in results if res.get("pic_id") == most_voted_pic_id), None)
                else:
                    final_pose_data = results[0] if results else None

                if final_pose_data:
                    pic_id_matched = final_pose_data.get("pic_id")
                    if pic_id_matched is not None and final_pose_data.get("localization_successful"):
                        stored_pose = await self.get_stored_node_pose(pic_id_matched)
                        if stored_pose:
                            stored_pose["pic_id"] = pic_id_matched
                            stored_pose["localization_successful"] = True
                            final_pose_data = stored_pose
                        else:
                            logger.warning(f"Failed to get stored pose for pic_id {pic_id_matched}, using initial localization data.")
                
                    # Add metadata to the final result
                    final_pose_data["image_name"] = image_name
                    final_pose_data["elapsed_ms"] = int((time.perf_counter() - start_time_total) * 1000)
                    logger.info(f"Successfully processed {image_name}. Final pose: {final_pose_data}")
                    return final_pose_data

                raise RuntimeError(f"Failed to aggregate localization results for {image_name}.")

            except (TimeoutError, RuntimeError, Exception) as e:
                logger.exception(f"Error processing {image_name}: {e}")
                return {
                    "error": f"{type(e).__name__}: {e}", 
                    "image_name": image_name, 
                    "elapsed_ms": int((time.perf_counter() - start_time_total) * 1000)
                }
            finally:
                # Always clean up the temporary directory
                if image_processing_dir.exists():
                    cleanup_dir(image_processing_dir)

    async def shutdown(self):
        logger.info("Shutting down RTAB-Map service...")

        # Close any cached database connections to release resources
        for db_path, conn in self.db_connection_cache.items():
            try:
                conn.close()
                logger.debug(f"Closed cached database connection to {db_path}")
            except Exception as e:
                logger.error(f"Error closing cached database connection to {db_path}: {e}")
        self.db_connection_cache.clear()
        self.node_poses_cache.clear()
        
        # Reset service state
        self.db_path = None
        self.is_initialized = False
        logger.info("RTAB-Map service shutdown complete.")

    async def preload_node_poses(self):
        """
        Preloads all node poses from the database for faster retrieval.
        """
        if not self.db_path:
            logger.warning("Cannot preload node poses: database path not set.")
            return

        # Get or create a database connection
        db_path_str = str(self.db_path)
        conn = self.db_connection_cache.get(db_path_str)
        if not conn:
            try:
                conn = sqlite3.connect(db_path_str)
                self.db_connection_cache[db_path_str] = conn
            except Exception as e:
                logger.error(f"Failed to connect to database {db_path_str} for preloading: {e}")
                return
        
        try:
            # Check if the 'Node' table exists before querying
            cursor = conn.cursor()
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='Node'")
            if not cursor.fetchone():
                logger.warning("Node table not found in database, cannot preload poses.")
                return
                
            # Clear any old cache and fetch all poses
            self.node_poses_cache.clear()
            start_time = time.perf_counter()
            cursor.execute("SELECT id, pose FROM Node")
            rows = cursor.fetchall()
            
            count = 0
            for node_id, pose_data in rows:
                # Parse and cache each pose
                parsed_pose = self._parse_and_format_pose(node_id, pose_data)
                if parsed_pose:
                    self.node_poses_cache[node_id] = parsed_pose
                    count += 1
            
            logger.info(f"Preloaded {count}/{len(rows)} node poses in {time.perf_counter() - start_time:.2f}s.")
        except Exception as e:
            logger.error(f"Error during node pose preloading: {e}")

def parse_localization_output(output: str):
    """
    Parse the output from rtabmap-console to extract localization pose and related info.
    Returns a dictionary with pose data and IDs, or None if parsing fails.
    """
    # Remove ANSI escape sequences for clean parsing
    ansi_escape = re.compile(r'\x1B\[[0-9;]*[A-Za-z]')
    cleaned_output = ansi_escape.sub('', output)
    logger.debug(f"Cleaned output:\n{cleaned_output}")

    # Prioritize finding the "Highest hypothesis" (pic_id) as it indicates a confident match.
    has_valid_match = False
    pic_id = None
    m_pic = re.search(r"Highest hypothesis\s*=\s*(\d+)", cleaned_output, re.IGNORECASE)
    if m_pic:
        try:
            pic_id = int(m_pic.group(1))
            has_valid_match = True # A high hypothesis means localization was successful
            logger.debug(f"Found valid highest hypothesis: {pic_id}")
        except Exception as e:
            logger.error(f"Error converting highest hypothesis token: {e}")
    else:
        logger.debug("No highest hypothesis found in output. Will use the initial localization pose.")

    # Always parse the "Localization pose" line, as it's present even on failure.
    match = re.search(r"Localization pose\s*=\s*([^\n\r]+(?:\n[^\n\r]+)?)", cleaned_output)
    if not match:
        logger.debug("No localization pose match found.")
        return None  # Cannot proceed without a pose.

    pose_str = match.group(1).strip()
    # Normalize the pose string: remove parentheses and commas for easier parsing
    pose_str = pose_str.replace('(', ' ').replace(')', ' ').replace(',', ' ')
    tokens = [tok for tok in pose_str.split() if tok]
    logger.debug(f"Raw pose tokens: {tokens}")
    map_id_val = None
    numeric_tokens = []
    
    # Extract numeric values and map_id from the pose string
    for token in tokens:
        # If the token includes '=' or ':', split it into key and value
        if '=' in token or ':' in token:
            key, val = re.split(r'[=:]', token, 1)
            if key.lower() in ("map", "map_id", "id"):
                try:
                    map_id_val = int(val.strip())
                except:
                    try:
                        map_id_val = int(float(val.strip()))
                    except Exception as e:
                        logger.error(f"Error converting map id token: {e}")
                continue # Skip adding to numeric_tokens
            else:
                numeric_tokens.append(val.strip()) # Add value part if it's not map_id
        else:
            numeric_tokens.append(token.strip())
            
    if len(numeric_tokens) < 6:
        logger.debug(f"Not enough numeric tokens for a valid pose (found {len(numeric_tokens)}: {numeric_tokens})")
        return None
    
    pose_tokens = numeric_tokens[:6]
    try:
        x, y, z, roll, pitch, yaw = map(float, pose_tokens)
    except Exception as e:
        logger.error(f"Error converting pose tokens to float: {e} (Tokens: {pose_tokens})")
        return None
        
    # Fallback to find map_id if not in the main pose string
    if map_id_val is None:
        m = re.search(r"\b(?:map[_\s]?id|id)\b\s*=\s*(\d+)", cleaned_output, re.IGNORECASE)
        if m:
            try:
                map_id_val = int(m.group(1))
            except Exception as e:
                logger.error(f"Error converting fallback map id token: {e}")
                
    # Another fallback for map_id from the numeric tokens
    if map_id_val is None:
        if len(numeric_tokens) > 6:
            try:
                map_id_val = int(float(numeric_tokens[6]))
            except (ValueError, IndexError):
                pass # Ignore if conversion fails
        if map_id_val is None:
             # As a last resort, try to find the last ID from the log
             m_last_id = re.findall(r"lastLocalizationPose=xyz=.*?id=(\d+)", cleaned_output)
             if m_last_id:
                 map_id_val = int(m_last_id[-1])

    precision = 5
    result = {
        "x": round(x, precision), 
        "y": round(y, precision), 
        "z": round(z, precision), 
        "roll": round(roll, precision), 
        "pitch": round(pitch, precision), 
        "yaw": round(yaw, precision), 
        "map_id": map_id_val,
        "localization_successful": has_valid_match  # Flag indicates if a confident match was found
    }
    
    if pic_id is not None:
        result["pic_id"] = pic_id
    
    logger.debug(f"Parsed pose: {result}")
    return result

def cleanup_dir(dir_path):
    """
    Safely remove a directory and all its contents.
    """
    try:
        if isinstance(dir_path, str):
            dir_path = Path(dir_path)
        
        if not dir_path.exists():
            return
        
        # Recursively remove all files and subdirectories
        for item in dir_path.iterdir():
            if item.is_dir():
                cleanup_dir(item)
            else:
                item.unlink()
        
        # Remove the now-empty directory
        try:
            dir_path.rmdir()
            logger.debug(f"Removed directory: {dir_path}")
        except Exception as e:
            logger.error(f"Error removing directory {dir_path}: {e}")
    except Exception as e:
        logger.error(f"Error cleaning up directory {dir_path}: {e}")

async def _ensure_service_initialized(db_path: Path):
    """
    Helper to initialize the RTAB-Map service for a given database if it's not already.
    """
    global rtabmap_service
    if not rtabmap_service:
        raise HTTPException(status_code=500, detail="RTAB-Map service not available.")
    
    # Initialize if the service is not running or if the DB has changed.
    if not rtabmap_service.is_initialized or rtabmap_service.db_path != db_path:
        logger.info(f"Initializing RTAB-Map service with database: {db_path}")
        if not await rtabmap_service.initialize(db_path):
            raise HTTPException(status_code=500, detail=f"Failed to initialize RTAB-Map with DB: {db_path.name}")

# --- API Endpoints ---

@app.get("/status")
async def service_status():
    """
    Returns detailed status information about the RTAB-Map service.
    """
    if not rtabmap_service:
        return {"status": "error", "message": "RTAB-Map service not created."}
        
    return {
        "status": "ok",
        "initialized": rtabmap_service.is_initialized,
        "database_path": str(rtabmap_service.db_path) if rtabmap_service.db_path else None,
        "cached_connections": len(rtabmap_service.db_connection_cache),
        "cached_node_poses": len(rtabmap_service.node_poses_cache),
        "timestamp": time.time()
    }

@app.get("/health")
async def health_check():
    """
    Simple health check to confirm the API is running.
    """
    return {
        "status": "ok",
        "service": "rtabmap-api",
        "version": "1.0",
        "timestamp": time.time()
    }

@app.post("/localize")
async def localize(image: UploadFile, db_name: str = Form(...)):
    """
    Main endpoint to localize an image against a specified RTAB-Map database.
    Accepts an image file and a database name as multipart form-data.
    """
    try:
        # Save the uploaded image to a temporary directory
        session_id = str(uuid.uuid4())
        session_dir = UPLOAD_BASE_DIR / f"session_{session_id}"
        session_dir.mkdir(parents=True, exist_ok=True)
        image_path = session_dir / image.filename
        with image_path.open("wb") as f:
            shutil.copyfileobj(image.file, f)

        # Resolve the full path to the database file
        db_path = DATA_DIR / "database" / db_name
        if not db_path.is_file():
            raise HTTPException(status_code=404, detail=f"Database '{db_name}' not found.")

        # Ensure the service is ready for the requested database
        await _ensure_service_initialized(db_path)

        # Process the image to get the localization pose
        result = await rtabmap_service.process_image(image_path)

        # If localization is successful, transform coordinates and send to Wayfinder
        if result and result.get("localization_successful"):
            transformed_coords = transform_coordinates(result["x"], result["y"])
            wayfinder_response = await send_to_wayfinder(transformed_coords["x"], transformed_coords["y"])
            result["wayfinder_update"] = wayfinder_response

        return result
    except Exception as e:
        logger.exception(f"An error occurred during localization: {e}")
        raise HTTPException(status_code=500, detail=f"An internal error occurred: {e}")
    finally:
        # Clean up the session directory
        if session_dir.exists():
            cleanup_dir(session_dir)
