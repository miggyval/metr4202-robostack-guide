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
## Installing `wget`
Ensure that `wget` is installed on your system.
### Apple / MacOS
```zsh
brew install wget
```
### Ubuntu
```bash
sudo apt install wget
```
## Installing Miniconda from Miniforge
Download the Miniconda install script with the following command.
```sh
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
```
Now, install miniconda from miniforge by running the script
### Ubuntu (with bash)
```bash
bash Miniforge3-$(uname)-$(uname -m).sh
```
### MacOS (with zsh)
```zsh
zsh Miniforge3-$(uname)-$(uname -m).sh
```
Follow the prompts to install conda on your system:
- Type `yes`
- Press `ENTER`
- Type `yes`

After this, you should have successfully installed conda.

Close and reopen your terminal.
# Step 2: Setting up the Mamba Environment
## Disable Auto-Activate base:
```sh
conda config --set auto_activate_base false
conda deactivate
```
## Install Mamba
```sh
conda install mamba -c conda-forge
```
Press `ENTER` to install `mamba` via `conda`.
## Create the mamba environment, and set up the channels.
Note that this is for ROS Humble Hawksbill.
```sh
mamba create -n ros_humble python=3.11
mamba activate ros_humble

conda config --env --add channels conda-forge
conda config --env --remove channels defaults
conda config --env --add channels robostack-humble
```
# Step 3: Install ROS Humble
For the minimal installation (for CLI/Headless use):
```sh
mamba install ros-humble-ros-base
```
For the desktop version (recommended if you have limited disk space)
```sh
mamba install ros-humble-desktop
```
For the full desktop version, with more packages pre-installed (recommended if you have ample disk space)
```sh
mamba install ros-humble-desktop-full
```
Press `ENTER` to confirm changes and install ROS2 Humble Hawksbill and relevant packages.
# Step 4: Verify Installation
After installing ROS Humble through mamba, verify the installation by running some basic commands.

Make sure that your terminal has the `ros_humble` environment activated.

If it doesn't show on the side like:
```sh
(ros_humble) user@my-pc:~$
```
Then run the following to activate.
```sh
mamba activate ros_humble
```
**Note: You will need to run this every time you open a new terminal**
## ROS CLI Commands
```sh
ros2 node list
```
```sh
ros2 topic list
```
```sh
ros2 pkg list

```
## ROS GUI Test
```sh
ros2 run rqt_gui rqt_gui

```
```sh
ros2 run rviz2 rviz2
```
## ROS Communication
Open two terminals, and run the following commands.
### Both Terminals
```sh
mamba activate ros_humble
```
### Terminal 1: Publisher / Talker
```sh
ros2 run demo_nodes_cpp talker
```
### Terminal 2: Subscriber / Listener
```sh
ros2 run demo_nodes_cpp listener
```
You should see the `talker` sending messages, and the `listener` receiving them.
```sh
(ros_humble) user@my-pc:~$ ros2 run demo_nodes_cpp listener
[INFO] [1752823777.564503365] [listener]: I heard: [Hello World: 3]
[INFO] [1752823778.564386061] [listener]: I heard: [Hello World: 4]
[INFO] [1752823779.564671590] [listener]: I heard: [Hello World: 5]
[INFO] [1752823780.564887598] [listener]: I heard: [Hello World: 6]
[INFO] [1752823781.564737702] [listener]: I heard: [Hello World: 7]
[INFO] [1752823782.565200646] [listener]: I heard: [Hello World: 8]
```
# Step 5: Development / Build Tools
Run the following command to install all necessary build tools for development in ROS.
```sh
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

# Step 6: Downgrade `setup-tools`
```sh
mamba install setuptools=58.0.4
```
# Step 7: Install Gazebo through Miniforge
```sh
mamba install -c conda-forge gazebo
```
```sh
mamba install ros-humble-gazebo-ros-pkgs
```
# Step 8: Shortcuts / Aliases
It's very useful to add aliases / shortcuts when developing to save time when typing.

Here are a few common aliases that you might find useful.

Create / edit a file in your home directory called `~/.bash_aliases` (`~/.zsh_aliases` for MacOS).
## Ubuntu (bash)
### Using `vim`
```sh
vim ~/.bash_aliases
```
### Using `nano`
```sh
nano ~/.bash_aliases
```
## MacOS (zsh)
### Using `vim`
```sh
vim ~/.zsh_aliases
```
### Using `nano`
```sh
nano ~/.zsh_aliases
```
## Aliases File
Copy and paste this into your aliases file, and restart your terminal to allow your changes to take effect.
```sh
alias humble="mamba deactivate; mamba activate ros_humble"
# For zsh (comment line if needed)
alias sws="source ./install/setup.zsh"
# For bash (uncomment line if needed)
alias sws="source ./install/setup.bash"
```
Now, when you type `humble` into your terminal, it will automatically activate the environment for ROS Humble.
