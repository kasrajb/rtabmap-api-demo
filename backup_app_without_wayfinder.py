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

# Configure logging for debugging and monitoring
logging.basicConfig(format="%(asctime)s [%(levelname)s] %(message)s", level=logging.DEBUG)
logger = logging.getLogger("rtabmap")
logger.setLevel(logging.DEBUG)

app = FastAPI()

# Enable CORS for all origins (for development and integration)
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

# Global variable to hold the RTABMap service instance
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
        self.base_rtabmap_params: List[str] = [] # Initialize as instance variable

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
            
            qx, qy, qz, qw = rotation_matrix_to_quaternion(r11, r12, r13, r21, r22, r23, r31, r32, r33)
            roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
            
            return {
                "x": round(tx, precision), "y": round(ty, precision), "z": round(tz, precision), 
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

            await self.shutdown() # Resets flags

            logger.info(f"Initializing RTAB-Map service with database: {db_path_obj}")
            if not db_path_obj.is_file():
                logger.error(f"Database file not found: {db_path_obj}")
                return False
            self.db_path = db_path_obj

            # Common parameters for rtabmap-console
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

            # Since daemon mode is not supported, we will always use per-image processing.
            # The service is now considered "initialized" for per-image processing.
            logger.info("RTAB-Map service will operate by launching rtabmap-console per image.")
            self.is_initialized = True
            await self.preload_node_poses()
            return True

    async def get_stored_node_pose(self, node_id: int) -> Union[dict, None]:
        """
        Retrieves the stored pose directly from the SQLite database, using caches.
        """
        if not self.db_path:
            return None
            
        if node_id in self.node_poses_cache:
            return self.node_poses_cache[node_id]
        
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
            cursor.execute("SELECT pose FROM Node WHERE id = ?", (node_id,))
            result = cursor.fetchone()
            
            if result:
                parsed_pose = self._parse_and_format_pose(node_id, result[0])
                if parsed_pose:
                    self.node_poses_cache[node_id] = parsed_pose
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
            self.image_counter += 1
            request_id = f"req_{self.image_counter}_{int(time.time())}"
            image_processing_dir = UPLOAD_BASE_DIR / f"img_proc_{request_id}"
            image_name = image_path.name
            start_time_total = time.perf_counter()
            
            try:
                image_processing_dir.mkdir(parents=True, exist_ok=True)
                target_image_in_processing_dir = image_processing_dir / image_name
                shutil.copy(image_path, target_image_in_processing_dir)

                # Simplified logic: always use per-image processing
                per_image_cmd = ["rtabmap-console", "-input", str(self.db_path)] + self.base_rtabmap_params + [str(image_processing_dir)]
                proc_img_loc = await asyncio.create_subprocess_exec(*per_image_cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                stdout_bytes, stderr_bytes = await asyncio.wait_for(proc_img_loc.communicate(), timeout=120.0)
                output_text_localization = stdout_bytes.decode(errors='ignore') + stderr_bytes.decode(errors='ignore')
                if proc_img_loc.returncode != 0:
                    logger.error(f"RTAB-Map per-image launch failed for {image_name}. RC={proc_img_loc.returncode}")

                initial_parsed_info = parse_localization_output(output_text_localization)
                final_pose_data = None
                
                pic_id_matched = initial_parsed_info.get("pic_id") if initial_parsed_info else None
                if pic_id_matched is not None:
                    final_pose_data = await self.get_stored_node_pose(pic_id_matched)
                    if final_pose_data:
                        final_pose_data["pic_id"] = pic_id_matched
                    else:
                        logger.warning(f"Failed to get stored pose for pic_id {pic_id_matched}, using initial localization data.")
                        final_pose_data = initial_parsed_info if initial_parsed_info and "x" in initial_parsed_info else None
                elif initial_parsed_info and "x" in initial_parsed_info:
                    logger.warning(f"No pic_id found for {image_name}, using initial localization pose.")
                    final_pose_data = initial_parsed_info
                
                if not final_pose_data:
                    raise RuntimeError(f"Could not determine final pose for {image_name}. Raw output: {output_text_localization[:1000]}")

                final_pose_data["image_name"] = image_name
                final_pose_data["elapsed_ms"] = int((time.perf_counter() - start_time_total) * 1000)
                logger.info(f"Successfully processed {image_name}. Final pose: {final_pose_data}")
                return final_pose_data

            except (TimeoutError, RuntimeError, Exception) as e:
                logger.exception(f"Error processing {image_name}: {e}")
                return {
                    "error": f"{type(e).__name__}: {e}", 
                    "image_name": image_name, 
                    "elapsed_ms": int((time.perf_counter() - start_time_total) * 1000)
                }
            finally:
                if image_processing_dir.exists():
                    cleanup_dir(image_processing_dir)

    async def shutdown(self):
        logger.info("Shutting down RTAB-Map service...")

        # Close any cached database connections
        for db_path, conn in self.db_connection_cache.items():
            try:
                conn.close()
                logger.debug(f"Closed cached database connection to {db_path}")
            except Exception as e:
                logger.error(f"Error closing cached database connection to {db_path}: {e}")
        self.db_connection_cache.clear()
        self.node_poses_cache.clear()
        
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
            cursor = conn.cursor()
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='Node'")
            if not cursor.fetchone():
                logger.warning("Node table not found in database, cannot preload poses.")
                return
                
            self.node_poses_cache.clear()
            start_time = time.perf_counter()
            cursor.execute("SELECT id, pose FROM Node")
            rows = cursor.fetchall()
            
            count = 0
            for node_id, pose_data in rows:
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

    # Look for a successful localization pose in the output
    match = re.search(r"Localization pose\s*=\s*([^\n\r]+(?:\n[^\n\r]+)?)", cleaned_output)
    if not match:
        logger.debug("No localization pose match found.")
        return None  # Cannot proceed without a pose.

    has_valid_match = True  # We found a pose, so localization was successful.

    # Try to find the "Highest hypothesis" (pic_id) for more accurate pose
    pic_id = None
    m_pic = re.search(r"Highest hypothesis\s*=\s*(\d+)", cleaned_output, re.IGNORECASE)
    if m_pic:
        try:
            pic_id = int(m_pic.group(1))
            logger.debug(f"Found valid highest hypothesis: {pic_id}")
        except Exception as e:
            logger.error(f"Error converting highest hypothesis token: {e}")
    else:
        logger.debug("No highest hypothesis found in output. Will use the initial localization pose.")

    pose_str = match.group(1).strip()
    # Normalize the pose string: remove parentheses and commas
    pose_str = pose_str.replace('(', ' ').replace(')', ' ').replace(',', ' ')
    tokens = [tok for tok in pose_str.split() if tok]
    logger.debug(f"Raw pose tokens: {tokens}")
    map_id_val = None
    numeric_tokens = []
    
    # If no valid match, we'll still parse the pose but will set all values to zero later
    for token in tokens:
        # If the token includes '=' or ':' then split it into key and value
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
                continue # Use continue to skip adding to numeric_tokens
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
        
    # If we found map_id from pose string, use it
    if map_id_val is None:
        m = re.search(r"\b(?:map[_\s]?id|id)\b\s*=\s*(\d+)", cleaned_output, re.IGNORECASE)
        if m:
            map_id_val = int(m.group(1))
            
    # More map_id fallback logic
    if map_id_val is None:
        if len(numeric_tokens) > 6:
            try:
                potential_map_id = int(float(numeric_tokens[6]))
                if str(potential_map_id) == numeric_tokens[6] or str(potential_map_id)+".0" == numeric_tokens[6]:
                    if not re.search(r"(?:map|map_id|id)\s*[:=]\s*" + re.escape(numeric_tokens[6]), cleaned_output, re.IGNORECASE):
                        logger.debug(f"Attempting to use token {numeric_tokens[6]} as map_id.")
                        map_id_val = potential_map_id
            except ValueError:
                pass
        if map_id_val is None:
             map_id_val = 0

    precision = 5
    result = {
        "x": round(x, precision), 
        "y": round(y, precision), 
        "z": round(z, precision), 
        "roll": round(roll, precision), 
        "pitch": round(pitch, precision), 
        "yaw": round(yaw, precision), 
        "map_id": map_id_val,
        "localization_successful": has_valid_match  # Add flag to indicate if this is a real match
    }
    
    if pic_id is not None:
        result["pic_id"] = pic_id
    
    logger.debug(f"Parsed pose: {result}")
    return result

def cleanup_dir(dir_path):
    """
    Safely remove a directory and all its contents (files and subdirectories).
    Used for cleaning up temporary session and processing directories.
    """
    try:
        if isinstance(dir_path, str):
            dir_path = Path(dir_path)
        
        if not dir_path.exists():
            return
        
        # Remove all files and subdirectories recursively
        for item in dir_path.iterdir():
            if item.is_dir():
                cleanup_dir(item)  # Recursively clean subdirectories
            else:
                try:
                    item.unlink()  # Remove file
                except Exception as e:
                    logger.warning(f"Failed to remove file {item}: {e}")
        
        # Remove the directory itself
        try:
            dir_path.rmdir()
            logger.debug(f"Removed directory: {dir_path}")
        except Exception as e:
            logger.warning(f"Failed to remove directory {dir_path}: {e}")
    except Exception as e:
        logger.error(f"Error cleaning up directory {dir_path}: {e}")

async def _ensure_service_initialized(db_path: Path):
    """
    Helper to initialize the RTAB-Map service for a given database if needed.
    Raises HTTPException if initialization fails.
    """
    global rtabmap_service
    if not rtabmap_service:
        raise HTTPException(status_code=500, detail="RTAB-Map service not available.")
    
    if not rtabmap_service.is_initialized or rtabmap_service.db_path != db_path:
        logger.info(f"Initializing RTAB-Map service with database: {db_path}")
        if not await rtabmap_service.initialize(db_path):
            raise HTTPException(status_code=500, detail="Failed to initialize RTAB-Map service.")

@app.get("/status")
async def service_status():
    """
    API endpoint: Returns detailed status information about the RTAB-Map service.
    Includes initialization state, database path, and cache sizes.
    """
    if not rtabmap_service:
        return {"status": "service_not_created", "initialized": False}
        
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
    API endpoint: Simple health check to verify the API is running.
    Returns status, service name, version, and timestamp.
    """
    return {
        "status": "ok",
        "service": "rtabmap-api",
        "version": "1.0.0",
        "timestamp": time.time()
    }

@app.post("/localize")
async def localize(request: Request):
    """
    API endpoint: Localize one or more images using a specified RTAB-Map database.
    Accepts JSON or multipart form data. Handles both single and batch image localization.
    Returns pose results for each image.
    """
    logger.debug("Received /localize request")
    if request.headers.get("content-length") and int(request.headers.get("content-length")) > 100 * 1024 * 1024:
        raise HTTPException(status_code=413, detail="Payload too large (max 100MB).")
    
    session_root = UPLOAD_BASE_DIR / f"session_{uuid.uuid4()}"
    image_files_to_process: List[Path] = []
    db_path_str = None
    is_single_image_request = False

    try:
        session_root.mkdir(parents=True, exist_ok=True)
        content_type = request.headers.get("content-type", "")

        if content_type.startswith("application/json"):
            data = await request.json()
            db_path_str = data.get("db_path")
            image_paths = data.get("image_paths") # Expect a list
            image_path = data.get("image_path")   # Support single path for backward compatibility
            image_dir_path = data.get("image_dir_path")

            if image_paths and isinstance(image_paths, list):
                is_single_image_request = len(image_paths) == 1
                image_files_to_process = [Path(p) for p in image_paths]
            elif image_path and isinstance(image_path, str):
                is_single_image_request = True
                image_files_to_process.append(Path(image_path))
            elif image_dir_path and isinstance(image_dir_path, str):
                is_single_image_request = False
                image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}
                image_files_to_process = [f for f in Path(image_dir_path).iterdir() if f.suffix.lower() in image_extensions]

        elif content_type.startswith("multipart/form-data"):
            form = await request.form()
            db_path_str = form.get("db_path")
            if not db_path_str:
                db_name = form.get("db_name")
                if db_name:
                    # Search for the db_name within the DATA_DIR
                    found_paths = list(DATA_DIR.rglob(f"**/{db_name}"))
                    if found_paths:
                        db_path_str = str(found_paths[0])
                        logger.info(f"Found db '{db_name}' at resolved path: {db_path_str}")
                    else:
                        # To maintain behavior and provide a clear error, we'll still construct the original path for the error message
                        db_path_str = str(DATA_DIR / db_name)
                        logger.warning(f"Could not find db '{db_name}' in any subdirectory of {DATA_DIR}. Will check original path.")

            # Note: multipart form only supports a single file upload in this logic
            upload_file = form.get("image") or form.get("file")
            if upload_file:
                is_single_image_request = True
                temp_img_path = session_root / upload_file.filename
                with open(temp_img_path, "wb") as f:
                    f.write(await upload_file.read())
                image_files_to_process.append(temp_img_path)

        if not db_path_str:
            raise HTTPException(status_code=400, detail="Request must include 'db_path'.")
        if not image_files_to_process:
            raise HTTPException(status_code=400, detail="No valid image(s) provided in 'image_path', 'image_paths', 'image_dir_path', or as a file upload.")

        db_path = Path(db_path_str)
        if not db_path.exists():
            raise HTTPException(status_code=404, detail=f"Database not found: {db_path_str}")
        
        await _ensure_service_initialized(db_path)

        tasks = [rtabmap_service.process_image(p) for p in image_files_to_process]
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        processed_results = []
        for i, res in enumerate(results):
            if isinstance(res, Exception):
                processed_results.append({"error": str(res), "image_name": image_files_to_process[i].name})
            else:
                processed_results.append(res)

        # Return a single object if the original request was for one image, otherwise a list
        if is_single_image_request:
            return processed_results[0] if processed_results else {"error": "Processing failed to return a result."}
        else:
            return {"results": processed_results, "count": len(processed_results)}

    except Exception as e:
        logger.exception(f"Error in /localize endpoint: {e}")
        if isinstance(e, HTTPException):
            raise e
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred: {e}")
    finally:
        if session_root.exists():
            cleanup_dir(session_root)

@app.on_event("startup")
async def startup_event():
    """
    FastAPI startup event: Initializes the RTABMap service when the application starts.
    """
    global rtabmap_service
    logger.info("Initializing RTABMap service...")
    rtabmap_service = RTABMapService()
    logger.info("RTABMap service initialized and ready for database loading.")

@app.on_event("shutdown")
async def shutdown_event():
    """
    FastAPI shutdown event: Cleans up resources and shuts down the RTABMap service.
    """
    global rtabmap_service
    if rtabmap_service:
        logger.info("Shutting down RTABMap service...")
        await rtabmap_service.shutdown()
        logger.info("RTABMap service shutdown complete.")
