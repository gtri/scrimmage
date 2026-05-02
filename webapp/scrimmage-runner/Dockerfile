FROM ghcr.io/gtri/scrimmage-24.04:latest

USER root

# Install Python + Flask for the launcher sidecar
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Set up venv to keep launcher deps isolated
RUN python3 -m venv /opt/launcher-venv

# Compose PATH: scrimmage install bin first, then launcher venv, then defaults.
# This image's base does NOT ship a setup.bash, so we set the scrimmage env vars
# explicitly here (matches share/cmake/scrimmage/setenv.in in the source).
ENV SCRIMMAGE_ROOT=/root/scrimmage/build/install
ENV PATH="${SCRIMMAGE_ROOT}/bin:/opt/launcher-venv/bin:${PATH}"
ENV LD_LIBRARY_PATH="${SCRIMMAGE_ROOT}/lib"
ENV SCRIMMAGE_PLUGIN_PATH="${SCRIMMAGE_ROOT}/lib/scrimmage/plugin_libs:${SCRIMMAGE_ROOT}/etc/scrimmage/plugins"
ENV SCRIMMAGE_MISSION_PATH="${SCRIMMAGE_ROOT}/share/scrimmage/missions"
ENV SCRIMMAGE_DATA_PATH="${SCRIMMAGE_ROOT}/share/scrimmage/data"
ENV SCRIMMAGE_CONFIG_PATH="${SCRIMMAGE_ROOT}/etc/scrimmage"

# The launcher's mission listing/templating reads from this dir
ENV MISSIONS_DIR=/root/scrimmage/missions

COPY launcher/requirements.txt /opt/launcher/requirements.txt
RUN pip install --no-cache-dir -r /opt/launcher/requirements.txt

COPY launcher/ /opt/launcher/
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

WORKDIR /opt

EXPOSE 50051 5050

ENTRYPOINT ["/opt/entrypoint.sh"]
