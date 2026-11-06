#!/usr/bin/env bash
# %% arguments and prerequisites
set -euo pipefail
repository=$(realpath "${1:-.}")
image=tiago-cram-real:local
container=aicor-cram-demo
for program in docker realpath; do
    command -v "$program" >/dev/null || { echo "Missing command: $program" >&2; exit 1; }
done
for required in .github/docker/Dockerfile uv.lock doc/tiago_cyclonedds.xml semantic_digital_twin/src/semantic_digital_twin/robots/tiago_single_arm.py semantic_digital_twin/resources/robots/tiago_single_arm/operator_exclusions.srdf; do
    test -f "$repository/$required" || { echo "Missing local project file: $required" >&2; exit 1; }
done
docker info >/dev/null
if docker container inspect "$container" >/dev/null 2>&1; then
    echo "Container $container already exists. No changes made; choose another name in this script." >&2
    exit 1
fi

# %% upstream ROS workspace image
DOCKER_BUILDKIT=1 docker build --tag tiago-cram-base:local "$repository/.github/docker"

# %% TIAGo runtime dependencies
build_directory=$(mktemp -d)
trap 'rm -rf "$build_directory"' EXIT
cat > "$build_directory/Dockerfile" <<'DOCKERFILE'
FROM tiago-cram-base:local
USER root
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    xvfb libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-render-util0 libxcb-xinerama0 libxcb-xkb1 \
    libxkbcommon-x11-0 libegl1 \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/pal-robotics/pal_urdf_utils.git /opt/ros/overlay_ws/src/pal_urdf_utils \
    && git -C /opt/ros/overlay_ws/src/pal_urdf_utils checkout e8600a339a3a9b7ffe67d1030190b97b67fb577b \
    && git clone https://github.com/pal-robotics/tiago_robot.git /opt/ros/overlay_ws/src/tiago_robot \
    && git -C /opt/ros/overlay_ws/src/tiago_robot checkout 5255eba8a74b7d9d6e5dab6d6e2b4c2fca886c97
RUN source /opt/ros/overlay_ws/install/setup.bash \
    && cd /opt/ros/overlay_ws \
    && colcon build --merge-install --executor sequential \
       --packages-select pal_urdf_utils tiago_description \
       --cmake-args -DBUILD_TESTING=OFF
WORKDIR /workspace/cram
DOCKERFILE
docker build --tag "$image" "$build_directory"

# %% development container with editable workspace
# DDS interface selection is enabled explicitly after Ethernet is configured.
docker run --detach --name "$container" --network host \
    --memory 8g --memory-swap 8g \
    --env OPENBLAS_NUM_THREADS=1 --env OMP_NUM_THREADS=1 \
    --env ROS_DOMAIN_ID=72 --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --mount "type=bind,source=$repository,target=/workspace/cram" \
    --workdir /workspace/cram "$image" sleep infinity

# %% Python environment and generated ORM interfaces
# Install through the project lockfile into its configured Python environment.
docker exec "$container" bash -c '
    set -e
    source /opt/ros/overlay_ws/install/setup.bash
    source /opt/ros/cram-env/bin/activate
    git config --global --add safe.directory /workspace/cram
    uv sync --locked --extra dev --active
    python scripts/regenerate_all_orm.py
    python -c "import sys; from semantic_digital_twin.robots.tiago_single_arm import TiagoSingleArm; from giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm import BaseVelocityLimits; print(sys.executable); print(TiagoSingleArm.get_ros_file_path()); print(BaseVelocityLimits())"
'

# %% completion
printf '\nCreated image %s and container %s. No robot commands sent.\n' "$image" "$container"
printf 'Open a shell with: docker exec -it %s bash\n' "$container"
printf 'Before ROS diagnostics, configure robot Ethernet and export inside the container:\n'
printf 'export CYCLONEDDS_URI=file:///workspace/cram/doc/tiago_cyclonedds.xml\n'
printf 'Dependencies outside the additional PAL pins follow the repository Dockerfile and uv.lock.\n'
