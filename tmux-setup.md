# TMUX
## What is tmux?

Short for terminal multiplexer, `tmux` [^1] allows different terminal programs to be run simultaneously in separate, organised terminal panels. This is especially useful for ROS/robotics applications as we normally have different programs running concurrently and might need them all active for troubleshooting purposes.

Unfortunately, for MacOS there are some intricacies with using `pixi` and `tmux` simultaneously, which is the reason why this tutorial exists. _If you have not already installed pixi, I encourage you to do that first - [Pixi setup](https://github.com/akash6001/ros2-macos/blob/main/pixi-setup.md)._

## Installing and using tmux

1. Installation is very simple. Ensure that Homebrew is installed and run the following:
   
   ```
   brew install tmux
   ```
2. To run tmux:
   
   ```
   tmux
   ```
   Once inside, there will be a green tab at the bottom of the screen to show that you have started the tmux session. To split or navigate panes the key shortcuts are:
   - `Ctrl+b "` - Split pane horizontally
   - `Ctrl+b %` - Split pane vertically
   - `Ctrl+b arrow key` - Navigate between panes
   
   note that the shortcuts are applied in 2 stages - `ctrl + b` first, followed by the subsequent key.

   ![Example 1](https://github.com/user-attachments/assets/d35bf775-a85f-4ac6-b901-d70d6a8f03e2)

   To close a tmux pane run:

   ```
   exit
   ```

## Using tmux with pixi

1. Go to a directory where a pixi environment has already been configured and installed. If you still have the directory from [Pixi setup](https://github.com/akash6001/ros2-macos/blob/main/pixi-setup.md), run:
   
   ```
   cd my_ros2_project
   ```

2. From here, always run `tmux` first (before running `pixi shell` or `pixi run`).

   ![Example 2](https://github.com/user-attachments/assets/ceb0e411-92de-411d-b6dd-03a0557f8347)   

[^1]: [tmux wiki](https://github.com/tmux/tmux/wiki)
