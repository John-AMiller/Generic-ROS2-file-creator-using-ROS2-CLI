# Generic-file-creator-using-ROS2-CLI
This repository includes a python script that creates a file within a ROS2 workspace while simultaneously updating the necessary CMake files.

# Implementation
This package must be placed in a ROS2 workspace within the src folder. Cloning the repository outside of the src folder of the ros2 workspace will not work as the package must be set in a true ROS2 workspace, which will not be created simply by cloning the repo. If you are only using this creator for a single package, you can edit the package name in the files create_file.py & create.py to the name of your package, which means you can avoid having to put the package name into the CLI as an argument every time you run the script, as it will already be in the file.  Just be sure you do this everywhere the package is referenced and that you are only creating nodes in that package. After installation of the repo, the package must be built and sourced (colcon build, source install/setup.bash).

# Usage
The usage of this program is simple, in a terminal run "ros2 create {workspace_name} {node_name} {language_name}" 
