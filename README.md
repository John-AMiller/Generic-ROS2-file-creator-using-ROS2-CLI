# Generic-file-creator-using-bash-script
This repository includes a bash script that creates a file within a ROS2 package while simultaneously updating the necessary CMakeList.txt, setup.py, and package.xml files.

# Usage
The usage of this program is simple, in a terminal run "./create_ros2_files.sh"
Doing so will bring up the following window:


![image](https://github.com/user-attachments/assets/b2333f7b-22c2-4c12-8cb0-c92e2ae00052)


Now, you can select the file type you want to create. Each file comes preloaded with default text, depending on the type of file the user selects. For example, selecting 1) Service creates a file that contains the following text for python:

![image](https://github.com/user-attachments/assets/b24540de-c114-43af-8237-d2d7ba82d910)

and this text for a C++ file type:

![image](https://github.com/user-attachments/assets/aaff6fa6-a18a-468a-a4b7-34bc04ff47d8)

The text that a file includes upon creation can easily be modified to the user's preferences by simply changing the text relating to that specific file type within "create_ros2_files". For example, changing the text in lines 63-84 (Assuming the user has made no edits to the file) will change the text in the Service type python file. The user can also make an empty file as the file does not need to contain any text upon creation for the script to work.

# Important Notes
Be sure that when entering the desired file name the user leaves the extension (.py or .cpp) ommited. Failure to do so will result in a file name similar to: file_name.py.py or file_name.cpp.cpp.
This script also assumes that the users ROS2 package is within a ROS2 workspace named "ros2_ws". If the users package is not within such a workspace they will need to edit the "create_ros2_files" file to change this. The lines that contain path designations that will need to be edited are: 52, 86, and any line containing "setup_file". (Again assuming the user has made no other edits to the file) 

![image](https://github.com/user-attachments/assets/05572187-30b0-4b1d-82d6-a744b7b3ed00)

Once the variables on these lines contain the correct path, the script will work again. 
