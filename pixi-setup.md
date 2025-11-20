# Setting up ROS2 with Pixi
## What is Pixi?

Pixi is a package manager that enables users to create virtual environments which contain different packages. ROS2 is one of the various packages which is supported by Pixi, and perhaps the simplest way to install and use ROS2 on MacOS is through Pixi.

## Installing Pixi

1. Open terminal and ensure that Homebrew[^1] is installed first. To check run:
   
   ```
   brew --version
   ```
   If installed, this should return something like `Homebrew 5.X.X`. If **not installed** run:
   
   ```
   curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh | bash
   ```
2. Once we have confirmed that Homebrew is installed, we can proceed to install Pixi.
   
   ```
   brew install pixi
   ```
   To check that Pixi has been successfully installed, run:
   
   ```
   pixi --version
   ```
   Should return something like `pixi 0.X.X`

3. I strongly recommend using the VSCode command line tools (assuming you are using VSCode). To do this,
   
   - Open VS Code
   - Open the Command Palette using `cmd + shift + P`
   - Click on `Shell Command: install 'code' command in PATH` (you might need to search it).
     
   ![VS Code shell command](https://github.com/user-attachments/assets/770cadeb-1a44-49fb-8157-1110619ef878)

   Once done, restart terminal and test the command.


## Setting up ROS2 using Pixi

1. We can now set up a Pixi workspace for ROS2. We will begin by creating a new directory for our ROS2 project.
   
   ```
   pixi init my_ros2_project
   ```
   This will create a new directory called `my_ros2_project` (of course, you can change the name if you want), which will contain a `pixi.toml` file. This file tells Pixi which packages to install in the workspace. To check, run:
   
   ```
   cd my_ros2_project
   ```
   followed by

   ```
   code pixi.toml
   ```

   _or if you don't want to use VS Code:_
   ```
   micro pixi.toml
   ```
   
   The file should look similar to this:
   
   ```toml
   [workspace]
   channels = ["conda-forge"]
   name = "my_ros2_project"
   platforms = ["osx-arm64"]
   version = "0.1.0"
   
   [tasks]
   
   [dependencies]

   ```

   
1. Now let's add our ROS2 configurations.

   ```toml
   [workspace]
   name = "my_ros2_project"
   channels = ["robostack-staging", "conda-forge"]
   platforms = ["osx-arm64"]
   
   [dependencies]
   ros-humble-desktop = "*"
   ros-humble-rclpy = "*"
   colcon-common-extensions = "*"
   python = "3.10.*"
   pip = "*"
   
   [activation.env]
   ROS_LOCALHOST_ONLY = "1"
   ROS_DOMAIN_ID = "0"

   ```

   What each line does:

   `[workspace]`
   - `channels = ["robostack-staging", "conda-forge"]` -> channels are like the sources for our packages, without them Pixi will not know where to find the packages to install (i.e. robostack-staging is the channel for our ros-humble-desktop package).
   
   `[dependencies]`
   - `ros-humble-desktop` -> installs the ros-humble-desktop package. the ` = "*"` line just tells Pixi to install all the submodules in the package.
   - `ros-humble-rclpy` -> installs the Python interface for ROS2 (RCLPY = ROS Client Library for Python); if you are using C++ for programming nodes, it would be `ros-humble-rclpp`.
   - `colcon-common-extensions` -> required for building our own packages (for ROS2).
   - `python = "3.10.*` -> any Python 3.10 version.
   - `pip` -> pip for installing additional Python packages if necessary.
  
   `[activation.env]`
   - `ROS_LOCALHOST_ONLY = "1"` -> ensures ROS nodes only communicate locally.
   - `ROS_DOMAIN_ID = "0"` -> domain ID can be anything, just allows us to identify a network of nodes.

2. Once we have finished editing the pixi.toml file, we are ready to install. Go back to terminal (ensure that you are in the correct directory, i.e. my_ros2_project) and run:
   
   ```
   pixi install
   ```
   This installs the packages as stated in the toml file.

## Using ROS2 with Pixi

1. Now that we have ROS2 installed, we are ready to start using it. Let's test it out by running the following:
   
   ```
   pixi run ros2 run turtlesim turtlesim_node
   ```
   Should spawn a random turtle. To exit, press `ctrl + c` while in the terminal.

2. Next, let's try building our own ROS2 package. To do this, we can use the Pixi shell.

   ```
   pixi shell
   ```
   Note that this command will only work in directories where the Pixi environment has already been set up.
   
   ```
   ros2 pkg create --build-type ament_python --destination-directory src --node-name my_ros_node my_ros_pkg
   ```
   Note that here, my_ros_node and my_ros_pkg are placeholder names (you can name them anything you want).

   To build the package, run:
   
   ```
   colcon build
   ```
3. Once the package has been built, to test it run:

   ```
   source install/setup.sh
   ```
   followed by
   
   ```
   ros2 run my_ros_pkg my_ros_node
   ```
   **Troubleshooting**

   If `ros2 run my_ros_pkg my_ros_node` gives "Package not found", try:
   - Make sure you ran `colcon build` and check for build errors.
   - Run `source install/setup.sh` in the same session before running ROS 2 commands.
   - Confirm that your package names match in both your folder structure and `ros2 run` command.

5. For the sake of convenience, we add the following to our `pixi.toml` file.
   ```toml
   [activation]
   scripts = ["install/setup.sh"]
   ```

   - By adding this line, we do not have to run `source install/setup.sh` every time we want to use ROS2 commands.
   - However, note that `source install/setup.sh` must still be run every time after running `colcon build`. The above only applies when testing the package without making any modifications.

[^1]: Package manager (system-level) for MacOS and Linux.
