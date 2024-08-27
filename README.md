# Generic-file-creator-using-ROS2-CLP
This repository includes a python script that creates a file within a ROS2 workspace while simultaneously updating the necessary CMake files.

# Implementation
The file must be placed in the ROS2 package exactly as it is in the repository. Cloning the repository will not work as the file must be set in a true ROS2 package, which will not be created simply by cloning the repo. You must copy the file into your own workspace, while making adjustments to the workspace name as needed. If you edit the workspace name in the file to the name of your workspace, you can avoid having to put the workspace name in to the CLP as an argument every time you run the script, as it will already be in the file. Just be sure you do this everywhere the workspace is referenced and that you are in the correct workspace.

