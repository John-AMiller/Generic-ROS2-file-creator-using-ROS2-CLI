import os
import sys

def create_python_node(package_name, node_name):
    package_path = os.path.join(os.environ['HOME'], 'ros2_ws', 'src', package_name)
    src_path = os.path.join(package_path, 'src')
    launch_path = os.path.join(package_path, 'launch')

    # Create the Python Node
    node_file = os.path.join(src_path, f"{node_name}.py")
    with open(node_file, 'w') as f:
        f.write(f"""
import rclpy
from rclpy.node import Node

class {node_name.capitalize()}(Node):

    def __init__(self):
        super().__init__('{node_name}')
        self.get_logger().info('Node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = {node_name.capitalize()}()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""")
    print(f"Python node {node_name} created at {node_file}.")

    # Update setup.py
    setup_file = os.path.join(package_path, 'setup.py')
    with open(setup_file, 'a') as f:
        f.write(f"\n        '{node_name} = {package_name}.{node_name}:main',")

    print(f"Node {node_name} added to setup.py.")

    # Create Launch File
    launch_file = os.path.join(launch_path, f"{node_name}_launch.py")
    with open(launch_file, 'w') as f:
        f.write(f"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='{package_name}',
            executable='{node_name}',
            name='{node_name}',
            output='screen'
        ),
    ])
""")
    print(f"Launch file created at {launch_file}.")

def create_cpp_node(package_name, node_name):
    package_path = os.path.join(os.environ['HOME'], 'ros2_ws', 'src', package_name)
    src_path = os.path.join(package_path, 'src')
    launch_path = os.path.join(package_path, 'launch')

    # Create the C++ Node
    node_file = os.path.join(src_path, f"{node_name}.cpp")
    with open(node_file, 'w') as f:
        f.write(f"""
#include "rclcpp/rclcpp.hpp"

class {node_name.capitalize()} : public rclcpp::Node
{{
public:
    {node_name.capitalize()}() : Node("{node_name}")
    {{
        RCLCPP_INFO(this->get_logger(), "Node has been started.");
    }}
}};

int main(int argc, char **argv)
{{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{node_name.capitalize()}>());
    rclcpp::shutdown();
    return 0;
}}
""")
    print(f"C++ node {node_name} created at {node_file}.")

    # Update CMakeLists.txt
    cmake_file = os.path.join(package_path, 'CMakeLists.txt')
    with open(cmake_file, 'a') as f:
        f.write(f"\nadd_executable({node_name} src/{node_name}.cpp)\nament_target_dependencies({node_name} rclcpp)\ninstall(TARGETS {node_name} DESTINATION lib/${{PROJECT_NAME}})")

    print(f"Node {node_name} added to CMakeLists.txt.")

    # Create Launch File
    launch_file = os.path.join(launch_path, f"{node_name}_launch.py")
    with open(launch_file, 'w') as f:
        f.write(f"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='{package_name}',
            executable='{node_name}',
            name='{node_name}',
            output='screen'
        ),
    ])
""")
    print(f"Launch file created at {launch_file}.")

def main():
    if len(sys.argv) != 4:
        print("Usage: create_node <package_name> <node_name> <language (cpp|py)>")
        return

    package_name = sys.argv[1]
    node_name = sys.argv[2]
    language = sys.argv[3]

    if language == 'py':
        create_python_node(package_name, node_name)
    elif language == 'cpp':
        create_cpp_node(package_name, node_name)
    else:
        print("Invalid language. Use 'cpp' or 'py'.")

if __name__ == '__main__':
    main()
