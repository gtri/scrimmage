FROM ghcr.io/gtri/scrimmage-24.04:latest

USER root

# Install Python + Flask for the launcher sidecar, plus C++ build tools for the overlay
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv \
        cmake g++ make \
        protobuf-compiler protobuf-compiler-grpc \
        libprotobuf-dev libgrpc++-dev \
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
ENV JSBSIM_ROOT="/root/scrimmage/data"

# c2overlay install root and plugin/library paths
ENV C2OVERLAY_ROOT=/opt/c2overlay/install
ENV SCRIMMAGE_PLUGIN_PATH="${C2OVERLAY_ROOT}/lib/c2overlay/plugin_libs:${C2OVERLAY_ROOT}/etc/c2overlay/plugins:${SCRIMMAGE_PLUGIN_PATH}"
ENV LD_LIBRARY_PATH="${C2OVERLAY_ROOT}/lib/c2overlay/plugin_libs:${LD_LIBRARY_PATH}"

# The launcher's mission listing/templating reads from this dir
ENV MISSIONS_DIR=/root/scrimmage/missions

# Build the c2overlay project (build context must be webapp/ — set in docker-compose)
COPY scrimmage-overlay /opt/c2overlay/src
RUN mkdir -p /opt/c2overlay/build && cd /opt/c2overlay/build \
    && cmake /opt/c2overlay/src \
         -DSETUP_LOCAL_CONFIG_DIR=OFF \
         -DCMAKE_INSTALL_PREFIX=${C2OVERLAY_ROOT} \
         -Dscrimmage_DIR=${SCRIMMAGE_ROOT}/lib/scrimmage/cmake \
    && make -j$(nproc) install

# Copy overlay missions into MISSIONS_DIR so they override the base image's copies
RUN mkdir -p "$MISSIONS_DIR" \
    && cp -f ${C2OVERLAY_ROOT}/share/c2overlay/missions/*.xml "$MISSIONS_DIR/"

COPY launcher/requirements.txt /opt/launcher/requirements.txt
RUN pip install --no-cache-dir -r /opt/launcher/requirements.txt

COPY launcher/ /opt/launcher/
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

WORKDIR /opt

EXPOSE 50051 5050 60001

ENTRYPOINT ["/opt/entrypoint.sh"]
