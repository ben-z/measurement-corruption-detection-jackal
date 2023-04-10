#!/bin/bash

set -o errexit -o nounset -o pipefail

HOST_DATA_ROOT="/hdd2"
HOST_PROFILE_PATH="$HOST_DATA_ROOT/.host_profile"

function install_docker() {
    # Set Docker data root to a larger drive
    __docker_data_root="$HOST_DATA_ROOT/docker_data_root"
    sudo mkdir -p "$__docker_data_root"
    sudo mkdir -p /etc/docker
    cat <<EOF | sudo tee /etc/docker/daemon.json
{
"data-root": "$__docker_data_root"
}
EOF

    __docker_config_dir="/tmp/docker-config"
    __docker_config_setter="export DOCKER_CONFIG=/tmp/docker-config"
    mkdir -p "$__docker_config_dir"
    # Add the config setter to host profile if it's not already there
    if ! grep -qF "$__docker_config_setter" "$HOST_PROFILE_PATH"; then
        echo "Adding '$__docker_config_setter' to $HOST_PROFILE_PATH"
        echo "$__docker_config_setter" >> "$HOST_PROFILE_PATH"
    fi
    source "$HOST_PROFILE_PATH"

    if ! command -v docker &> /dev/null; then
        # https://docs.docker.com/engine/install/debian/#install-using-the-convenience-script
        curl -fsSL https://get.docker.com | sh

        # https://docs.docker.com/engine/install/linux-postinstall/
        sudo usermod -aG docker $USER
        newgrp docker
    fi
}

mkdir -p "$HOST_DATA_ROOT"
sudo chmod 777 "$HOST_DATA_ROOT"

# ====================================================
# Configure host profile
# ====================================================
echo "========================================"
echo "Setting up host profile at $HOST_PROFILE_PATH"
touch "$HOST_PROFILE_PATH"
__host_profile_setter="[ -f $HOST_PROFILE_PATH ] && source $HOST_PROFILE_PATH"
if ! grep -qF "$__host_profile_setter" ~/.bashrc; then
    echo "Adding '$__host_profile_setter' to ~/.bashrc"
    echo "$__host_profile_setter" >> ~/.bashrc
fi

# ====================================================
# Install Docker
# ====================================================
echo "========================================"
echo "Installing Docker"
install_docker

# ====================================================
# Install development tools
# ====================================================
echo "========================================"
echo "Installing development tools"
sudo apt-get update
sudo apt-get install -y moreutils
