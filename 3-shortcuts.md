## Writing a script to automate the ROS 2 environment setup

Assuming you have already gone through the steps in [Pixi setup](https://github.com/akash6001/ros-stuff/blob/main/1-pixi-setup.md), you might have noticed that there is a very long list of actions to take in order to setup a ROS environment with Pixi. The question now is, is it possible to automate everything?

And the answer is yes. We can do this by means of a shell script.

1. Open your default zshrc script with:

   ```
   code ~/.zshrc
   ```
   _or if you are not using VS Code:_
   ```
   pico ~/.zshrc
   ```

2. Scroll to the bottom of the file and paste the following:
   
   ```shell
    ros-new() {
        local project_name=$1
        
        if [ -z "$project_name" ]; then
            echo "Error: Please provide a project name"
            echo "Usage: ros-new <project_name>"
            return 1
        fi

        target_dir=~/ros_projects/$project_name
   
        # Check if directory already exists
        if [ -d "$target_dir" ]; then
            echo "⚠️  Warning: Directory $target_dir already exists."
            echo -n "Do you want to overwrite it? (y/n): "
            read answer
            case "$answer" in
                [Yy]* )
                    rm -rf "$target_dir"
                    echo "Directory overwritten."
                    echo ""
                    ;;
                * )
                    echo "Aborting. No changes made."
                    return 1
                    ;;
            esac
        fi


        echo "🚀 Creating ROS 2 Humble project: $project_name"
        echo "📦 Using channel: robostack-staging"
        echo ""
        
        # Create directory structure
        mkdir -p "$target_dir/src"
        cd "$target_dir"
        
        # Initialize pixi (== pixi init)
        cat > pixi.toml << EOF
    [workspace]
    name = "$project_name"
    version = "0.1.0"
    description = "ROS 2 Humble dev environment"
    authors = ["Your Name <example@gmail.com>"]
    channels = ["robostack-staging", "conda-forge"]
    platforms = ["osx-arm64", "osx-64"]

    [dependencies]
    ros-humble-desktop = "*"
    ros-humble-rclpy = "*"
    colcon-common-extensions = "*"
    graphviz = "*"

    [activation]
    scripts = ["dev.sh"]

    [activation.env]
    ROS_LOCALHOST_ONLY = "1"
    ROS_DOMAIN_ID = "0"
   EOF

        echo "📦 Installing dependencies..."
        pixi install
        
        # Create setup script
        cat > dev.sh << 'EOF'
    #!/bin/bash
    set -e

    if [ -f install/setup.sh ]; then
        . install/setup.sh
        echo "✅ ROS + workspace ready!"
    else
        echo "✅ ROS ready"
    fi
   EOF
        
        chmod +x dev.sh
        echo ""

        # Create .gitignore
        cat > .gitignore << 'EOF'
    # Pixi environment
    .pixi/
    pixi.lock

    # ROS build artifacts
    build/
    install/
    log/

    # Python
    __pycache__/
    *.pyc
    *.pyo

    # OS-specific
    .DS_Store
   EOF
        
        echo "✅ Project '$project_name' created successfully!"
        echo "💾 ROS Distribution: Humble"
        echo "📁 Location: $(pwd)"
        echo ""
    }
   ```

   **Notes**
   - What this does is create a shell command called `ros-new` that can be used to start new ROS 2 projects from scratch. The script creates the pixi.toml file and installs the necessary dependencies in the Pixi environment. Once the directory has been created, all you need to do is enter Pixi shell to start using ROS 2 commands.
  
3. You can test it out by running the following command in your terminal (you will need to restart the terminal after editing the ~/.zshrc file).

   ```
   ros-new test_project
   ```

   Note that you can replace `test_project` with any name, that is the whole point.
