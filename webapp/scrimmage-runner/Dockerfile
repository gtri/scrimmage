FROM ghcr.io/gtri/scrimmage-24.04:latest

USER root

# Install Python + Flask for the launcher sidecar
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Set up venv to keep launcher deps isolated
RUN python3 -m venv /opt/launcher-venv
ENV PATH="/opt/launcher-venv/bin:${PATH}"

COPY launcher/requirements.txt /opt/launcher/requirements.txt
RUN pip install --no-cache-dir -r /opt/launcher/requirements.txt

COPY launcher/ /opt/launcher/
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

# Make sure scrimmage's env is sourced for child processes
ENV MISSIONS_DIR=/root/scrimmage/scrimmage/missions

EXPOSE 50051 5050

ENTRYPOINT ["/opt/entrypoint.sh"]
