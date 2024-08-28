# Generic-file-creator-using-ROS2-CLI
This repository includes a python script that creates a file within a ROS2 workspace while simultaneously updating the necessary CMake files.

# Implementation
The file must be placed in the ROS2 workspace exactly as it is in the repository, within the src folder. Cloning the repository outside of the src folder of the ros2 workspace will not work as the files must be set in a true ROS2 workspace, which will not be created simply by cloning the repo. If you are only using this creator for a single package, you can edit the workspace name in the files (create_file.py & create.py) to the name of your workspace, you can avoid having to put the workspace name into the CLI as an argument every time you run the script, as it will already be in the file. Just be sure you do this everywhere the workspace is referenced and that you are in the correct workspace. After installation of the repo, the package must be rebuilt and sourced (colcon build, source install/setup.bash)

# Usage
The usage of this program is simple, in a terminal run "ros2 create {workspace_name} {node_name} {language_name}" 
