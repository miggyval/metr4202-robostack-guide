# METR4202 Robostack Setup Guide
This repository is intended for METR4202 students who are using the following supported devices:
- **MacOS (Apple Silicon / ARM64)** running MacOS 11.0 (Big Sur) or newer
- **MacOS (Intel / AMD64)** running MacOS 10.13 (High Sierra) or newer
- **Ubuntu 22.04 / Ubuntu 24.04 (AMD64 / x86_64)**
- **Ubuntu 22.04 / Ubuntu 24.04 (ARM64 / aarch64)**
If you are unsure, please ask your tutor in your session.

# Step 0: Installing Homebrew (Apple / MacOS only)
Open your Terminal.
```zsh
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

# Step 1: Installing Conda (Miniconda)
Open your Terminal.
Ensure that `wget` is installed on your system.
## Apple / MacOS
```zsh
brew install wget
```
## Ubuntu
```bash
sudo apt install wget
```
Now, install miniconda from miniforge with the following command.
```sh
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
```
Follow the prompts to install conda on your system.
Close and reopen your terminal.
# Step 2: Setting up the Mamba Environment
## Disable Auto-Activate base:
```sh
conda config --set auto_activate_base false
conda deactivate
```
## Install Mamba
```
conda install mamba -c conda-forge
```
## Create the mamba environment, and set up the channels.
Note that this is for ROS Humble Hawksbill.
```sh
mamba create -n ros_humble
mamba activate ros_humble

conda config --env --add channels conda-forge
conda config --env --remove channels defaults
conda config --env --add channels robostack-humble
```
# Step 3: Install ROS Humble
For the minimal installation, run:
```sh
mamba install ros-humble-ros-base
```
For the desktop version (recommended if you are low on disk space)
```sh
mamba install ros-humble-desktop
```
For the full desktop version, with more packages pre-installed (recommended if you have lots of disk space)
```sh
mamba install ros-humble-desktop-full
```

