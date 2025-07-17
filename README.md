# RTAB-Map Localization API

[RTABMap API Demo](https://github.com/kasrajb/rtabmap-api-demo)

This project provides a containerized RESTful API (built with FastAPI in Python 3.8) to perform image localization using RTAB-Map’s command-line tool and direct database access. The API accepts an input image and a pre-built RTAB-Map database (map) and returns the 6-DoF pose (x, y, z, roll, pitch, yaw) of the image within that map. Additionally, it integrates with the Wayfinder system to update coordinates.

## Features

- **Endpoints:**
  - `POST /localize` – Accepts an image (multipart/form-data) and a database name, runs RTAB-Map’s localization, transforms coordinates to the Wayfinder system, and returns a JSON response:
    ```json
    { "x": ..., "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ..., "map_id": ..., "pic_id": ..., "elapsed_ms": ..., "wayfinder_update": ... }
    ```
    Example request (multipart/form-data):
    ```bash
    curl -X POST -F "image=@test.jpg" -F "db_name=my_map.db" http://localhost:8000/localize
    ```
    Example response:
    ```json
    { "x": 1.23, "y": 4.56, "z": 7.89, "roll": 0.12, "pitch": 0.34, "yaw": 0.56, "map_id": 42, "pic_id": 101, "elapsed_ms": 1234, "wayfinder_update": { "status": "success" } }
    ```
  - `GET /health` – Returns `{"status": "ok"}` to indicate the service is running.
  - `GET /status` – Provides detailed information about the RTAB-Map service, including initialization state, database path, and cache sizes.

- **RTAB-Map Integration:**
  - The API uses `rtabmap-console` for initial image localization against a database. Parameters like `Mem/IncrementalMemory=false` and `InitWMWithAllNodes=true` ensure pure localization.
  - Precise stored pose retrieval is done via direct SQLite database queries, bypassing further `rtabmap-console` calls for improved speed and reliability.

- **Wayfinder System Integration:**
  - Transforms RTAB-Map coordinates into the Wayfinder system using pre-calculated rotation and translation parameters.
  - Sends transformed coordinates to the Wayfinder API for updates.

- **Utility Functions:**
  - Quaternion and rotation matrix conversions for pose calculations.
  - File management utilities for handling uploads and temporary files.
  - Directory cleanup functions to safely remove temporary processing directories.

- **Performance & Concurrency:**
  - Handles multiple requests concurrently. Each localization request spawns a separate `rtabmap-console` process (via `asyncio.create_subprocess_exec`).
  - Cached database connections and preloaded node poses improve efficiency.

## Using Localization Data to Proceed to a Destination

1. **Retrieve Pose Data:**
   - Use the `POST /localize` endpoint to process an image and retrieve pose data.
   - The response includes coordinates (`x`, `y`, `z`) and orientation (`roll`, `pitch`, `yaw`).

2. **Transform Coordinates:**
   - The API automatically transforms RTAB-Map coordinates into the Wayfinder system.

3. **Update Wayfinder System:**
   - The transformed coordinates are sent to the Wayfinder system for updates.

## Container Architecture

This solution uses Docker Compose.
The primary service is **`rtabmap-api`**:
- Built from the provided `Dockerfile`, which uses `introlab3it/rtabmap:focal` as its base. This means the `rtabmap-api` container **includes all necessary RTAB-Map command-line tools**.
- Runs the FastAPI application (with Uvicorn) as a non-root user (`appuser`).
- Mounts a shared `/data` volume to access map databases and process images.
- The FastAPI app invokes `rtabmap-console` for initial localization and uses Python's `sqlite3` module for direct database queries.

(Optional: If you keep the `rtabmap-core` service in `docker-compose.yml`)
An additional **`rtabmap-core`** service (Image: `introlab3it/rtabmap:focal`) can be used primarily for data volume management or to run other RTAB-Map tasks (e.g., map generation) independently, sharing the `/data` volume.

**Security:** The API container runs as a non-root user (`appuser`) and only exposes port 8000. The shared volume is restricted to map data and images. Ensure that only trusted clients call the API. The API validates input paths to prevent arbitrary file access outside `/data`.

## Building and Running

**Prerequisites:** Install Docker and Docker Compose. Place your RTAB-Map database (e.g., `my_map.db`) in the `data` subfolder of this project (e.g., `c:\rtabmap-api-demo\data\my_map.db`), which is bind-mounted to `/data` in the containers.

**Build and run the services:**
```powershell
# Build and start the services
docker-compose up -d --build
```
To view logs:
```powershell
# View logs for the API service
docker-compose logs -f rtabmap-api
```
When done:
```powershell
# Stop and remove the services
docker-compose down
```

## Development Notes

- **Code Structure:**
  - `app.py`: Main FastAPI application with endpoints and utility functions.
  - `initialize_wayfinder.py`: Script for initializing the Wayfinder module.
  - `docker-compose.yml` and `Dockerfile`: Containerization setup.
  - `requirements.txt`: Python dependencies.

- **Testing:**
  - Use the provided test images in `data/test/` for localization testing.
  - Ensure the database files in `data/database/` are accessible and correctly formatted.

- **Logs:**
  - Application logs are stored in `LogF.txt` and `LogI.txt` for debugging and monitoring purposes.

## Future Improvements

- Add more endpoints for advanced RTAB-Map functionalities.
- Optimize database queries for larger datasets.
- Enhance security measures for production deployment.
- Implement real-time localization using streaming data.
- Add support for additional RTAB-Map parameters and configurations.

---

## How to Set Up the System

1. **Clone the Repository**:
   Open a terminal and run the following command:
   ```bash
   git clone https://github.com/kasrajb/rtabmap-api-demo.git
   ```

2. **Navigate to the Repository**:
   ```bash
   cd rtabmap-api-demo
   ```

3. **Install Dependencies**:
   Follow the instructions provided in the repository to install any required dependencies.

---

## How to Update the System

1. **Pull the Latest Changes**:
   Each time the code is updated, run the following command to fetch and merge the latest changes:
   ```bash
   git pull origin main
   ```
   Replace `main` with the name of the default branch if it's different.

2. **No Need to Rebuild**:
   Updates are automatically applied, and there is no need to rebuild the system unless major changes are made to the container or dependencies.

---

## If You Make Changes

1. **Commit and Push Changes**:
   After making changes, use the following commands to commit and push them to the repository:
   ```bash
   git add .
   git commit -m "Description of changes"
   git push origin main
   ```

