# Setting up ROS2 with Pixi
## What is Pixi?

Pixi is a package manager that enables users to create virtual environments which contain different packages. ROS2 is one of the various packages which is supported by Pixi, and perhaps the simplest way to install ROS2 on MacOS is by installing Pixi first.

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

3. I strongly recommend using the VSCode command line tools (assuming you are using VSCode).
   

## Setting up ROS2 using Pixi

1. We can now set up a Pixi workspace for ROS2. We will begin by creating a new directory for our ROS2 project.
   
   ```
   pixi init my_ros2_project
   ```
   This will create a new directory called `my_ros2_project` (of course, you can change the name if you want), which will contain a `pixi.toml` file. This file tells Pixi which packages to install in the workspace. To check, run:
   
   ```
   cd my_ros2_project
   ```
   
2. Now let's add ROS2.

   ```
   pixi add 
   ```
   
[^1]: Package manager (system-level) for MacOS and Linux.
