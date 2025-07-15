# ────────────────────────────────────────────────────────────────────────────
# 1. Base image ‒ already contains RTAB-Map CLI binaries
# ────────────────────────────────────────────────────────────────────────────
# Use the newer Ubuntu 22.04 (Jammy) based image for daemon support
FROM introlab3it/rtabmap:jammy

# Suppress interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# ────────────────────────────────────────────────────────────────────────────
# 2. Install Python 3 and pip inside the Ubuntu image
# ────────────────────────────────────────────────────────────────────────────
# Install Python 3, pip, and setuptools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-setuptools && \
    pip3 install --no-cache-dir --upgrade pip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ────────────────────────────────────────────────────────────────────────────
# 3. Copy requirements and install them
# ────────────────────────────────────────────────────────────────────────────
# Set the working directory to /app
WORKDIR /app

# Copy requirements and install them
COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

# ────────────────────────────────────────────────────────────────────────────
# 4. Add unprivileged user and copy application code
# ────────────────────────────────────────────────────────────────────────────
# Add unprivileged user and copy application code
RUN useradd -ms /bin/bash appuser
COPY . /app
RUN mkdir /data
RUN chown -R appuser:appuser /data
RUN chown -R appuser:appuser /app

# Switch to the unprivileged "appuser"
USER appuser

# ────────────────────────────────────────────────────────────────────────────
# 5. Expose port and launch FastAPI via Uvicorn
# ────────────────────────────────────────────────────────────────────────────
# Expose port 8000 for the API
EXPOSE 8000

# Start the FastAPI app
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]
