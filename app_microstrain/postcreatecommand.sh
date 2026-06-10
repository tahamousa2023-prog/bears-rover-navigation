#!/bin/bash
set -e

echo "Running postCreateCommand.sh ..."

# Go to workspace
cd /workspace

# Ensure src folder exists
mkdir -p src

# Initialize rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Fix ament_python package libexec directory issue
if [ -d "install/app_template" ]; then
    mkdir -p install/app_template/lib/app_template
    if [ -f "install/app_template/bin/template_node" ]; then
        ln -sf ../../bin/template_node install/app_template/lib/app_template/template_node
    fi
fi

# Source environment setup for all shells
echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set CycloneDDS as default RMW (since you installed it)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

echo "✅ Post-create setup complete."
