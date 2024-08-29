# Generic-file-creator-using-ROS2-CLI
This repository includes a python script that creates a file within a ROS2 workspace while simultaneously updating the necessary CMake files.

# Implementation


# Usage
The usage of this program is simple, in a terminal run "./create_ros2_files.sh"
Doing so will bring up the following window:


![image](https://github.com/user-attachments/assets/b2333f7b-22c2-4c12-8cb0-c92e2ae00052)


Now, you can select the file type you want to create. Each file comes preloaded with default text, depending on the type of file the user select. For example, selecting 1) Service creates a file that contains the following text for python:

![image](https://github.com/user-attachments/assets/b24540de-c114-43af-8237-d2d7ba82d910)

and this text for a C++ file type:

![image](https://github.com/user-attachments/assets/aaff6fa6-a18a-468a-a4b7-34bc04ff47d8)

This text can easily be modified to the user's preferences by simply changing the text relating to that specific file type within create_ros2_files. The user can also make an empty file as the file does not need to contain any text upon creation for the script to work.

# Notes
Be sure that when entering the desired file name the user leaves the extension (.py or .cpp) ommited. Failure to do so will result in a file name similar to: file_name.py.py or file_name.cpp.cpp.
