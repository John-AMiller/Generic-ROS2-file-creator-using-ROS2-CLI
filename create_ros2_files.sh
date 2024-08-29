#!/bin/bash

echo "What do you want to create?"
echo "1) Service"
echo "2) Action"
echo "3) Client"
echo "4) Publisher"
echo "5) Subscriber"

read -p "Enter your choice: " choice 

case $choice in
    1)
        echo "creating a Service..."
        read -p "Do you want to create a launch file for your Service? (y/n)" create_launch
        ;;
    2)
        echo "Creating an Action..."
        read -p "Do you want to create a launch file for your Action ? (y/n)" create_launch
        ;;
    3) 
        echo "Creating a Client..."
        read -p "Do you want to create a launch file for your Client? (y/n)" create_launch
        ;;
    4)
        echo "Creating a Publisher..."
        read -p "Do you want to create a launch file for your Publisher? (y/n)" create_launch
        ;;
    5)
        echo "Creating a Listener..."
        read -p "Do you want to create a launch file for your Listener? (y/n)" create_launch
        ;;
    *)
        echo "Invalid. Exiting"
        exit 1
        ;;
esac

read -p "Package: " package_name
read -p "File name (NoExtension):" file_name
read -p "Language: " language
    
    if [ "$language" == "py" ]; then
        extension="py"
    elif [ "$language" == "cpp" ]; then
        extension="cpp"
    else 
        echo "Invalid language. Exiting"
        exit 1
    fi

file_path=~/ros2_ws/src/$package_name
SRC_PATH=$file_path
launch_path=$package_name/launch

case $choice in
  1)
    # Service Template
    if [ "$language" == "py" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class $file_name(Node):

    def __init__(self):
        super().__init__('$file_name')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a=%d b=%d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = $file_name()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
        setup_file=~/ros2_ws/src/$package_name/setup.py
        if ! grep -q "$file_name" $setup_file; then
            sed -i "/console_scripts/a \  '$file_name = $package_name.$file_name:main'," $setup_file
        fi

    elif [ "$language" == "cpp" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class $file_name : public rclcpp::Node
{
public:
    $file_name() : Node("$file_name")
    {
        auto service = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&${file_name}::add_two_ints_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void add_two_ints_callback(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                               std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld b=%ld", request->a, request->b);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<$file_name>());
    rclcpp::shutdown();
    return 0;
}
EOF

    cmake_file=~/ros2_ws/src/$package_name/CMakeLists.txt
    if ! grep -q "$file_name" $cmake_file; then
        echo "add_executable($file_name src/file_name.cpp)" >> $cmake_file
        echo "ament_target_dependencies($file_name rclcpp)" >> $cmake_file
        echo "install(TARGETS $file_name DESTINATION lib/\${PROJECT_NAME})" >> $cmake_file
        fi
    fi

    package_file=~/ros2_ws/src/$package_name/package.xml
    if ! grep -q "<exec_depend>example_interfaces</exec_depend>" $package_file; then
        sed -i "/<\/build_depend>/a \ <exec_depend>example_interfaces</exec_depend>" $package_file
    fi

    if [ "$create_launch" == "y" ]; then
        launch_file=~/ros2_ws/src/$package_name/launch/${file_name}_launch.py
        mkdir -p ~/ros2_ws/src/$package_name/launch
        cat << EOF > $launch_file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='$package_name',
            executable='$file_name',
            name='$file_name',
            output='screen'
        ),
    ])
EOF
        echo "Launch file created at $launch_file"
    fi
    ;;
  2)
        if [ "$language" == "py" ]; then
            NODE_FILE="$SRC_PATH/$file_name.$extension"
            cat << EOF > $NODE_FILE
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node



class ""(Node):

    def __init__(self):
        super().__init__('$file_name')
        self._action_server = ActionServer(
            self,
            "",
            '""',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = "".Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    $file_name = ""()

    rclpy.spin($file_name)


if __name__ == '__main__':
    main()
EOF
        setup_file=~/ros2_ws/src/$package_name/setup.py
        if ! grep -q "$file_name" $setup_file; then
            sed -i "/console_scripts/a \  '$file_name = $package_name.$file_name:main'," $setup_file
        fi

    elif [ "$language" == "cpp" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace action_tutorials_cpp
{
class "" : public rclcpp::Node
{
public:
  using "" = action_tutorials_interfaces::action::"";
  using GoalHandle"" = rclcpp_action::ServerGoalHandle<"">;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit ""(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("$file_name", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<"">(
      this,
      """",
      std::bind(&""::handle_goal, this, _1, _2),
      std::bind(&""::handle_cancel, this, _1),
      std::bind(&""::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<"">::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ""::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle""> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle""> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&""ActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle""> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<""::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<""::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class ""

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::"")
EOF
    cmake_file=~/ros2_ws/src/$package_name/CMakeLists.txt
    if ! grep -q "$file_name" $cmake_file; then
        echo "add_executable($file_name src/file_name.cpp)" >> $cmake_file
        echo "ament_target_dependencies($file_name rclcpp)" >> $cmake_file
        echo "install(TARGETS $file_name DESTINATION lib/\${PROJECT_NAME})" >> $cmake_file
        fi
    fi

    package_file=~/ros2_ws/src/$package_name/package.xml
    if ! grep -q "<exec_depend>example_interfaces</exec_depend>" $package_file; then
        sed -i "/<\/build_depend>/a \ <exec_depend>example_interfaces</exec_depend>" $package_file
    fi

    if [ "$create_launch" == "y" ]; then
        launch_file=~/ros2_ws/src/$package_name/launch/${file_name}_launch.py
        mkdir -p ~/ros2_ws/src/$package_name/launch
        cat << EOF > $launch_file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='$package_name',
            executable='$file_name',
            name='$file_name',
            output='screen'
        ),
    ])
EOF
        echo "Launch file created at $launch_file"
    fi
    ;;
  3)
    # Client Template
    if [ "$language" == "py" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class $file_name(Node):

    def __init__(self):
        super().__init__('$file_name')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(input("Enter a: "))
        self.req.b = int(input("Enter b: "))
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = $file_name()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF 
        setup_file =~/ros2_ws/src/$package_name/setup.py
        if ! grep -q "$file_name" $setup_file; then
            sed -i "/console_scripts/a \  '$file_name = $package_name.$file_name:main'," $setup_file
        fi

    elif [ "$language" == "cpp" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>

using namespace std::chrono_literals;

class $file_name : public rclcpp::Node
{
public:
    $file_name() : Node("$file_name")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        request_ = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    }

    void send_request()
    {
        request_->a = 2;
        request_->b = 3;
        auto result = client_->async_send_request(request_);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Result: %d", result.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<$file_name>();
    node->send_request();
    rclcpp::shutdown();
    return 0;
}
EOF
    cmake_file=~/ros2_ws/src/$package_name/CMakeLists.txt
    if ! grep -q "$file_name" $cmake_file; then
        echo "add_executable($file_name src/file_name.cpp)" >> $cmake_file
        echo "ament_target_dependencies($file_name rclcpp)" >> $cmake_file
        echo "install(TARGETS $file_name DESTINATION lib/\${PROJECT_NAME})" >> $cmake_file
        fi
    fi

    package_file=~/ros2_ws/src/$package_name/package.xml
    if ! grep -q "<exec_depend>example_interfaces</exec_depend>" $package_file; then
        sed -i "/<\/build_depend>/a \ <exec_depend>example_interfaces</exec_depend>" $package_file
    fi

    if [ "$create_launch" == "y" ]; then
        launch_file=~/ros2_ws/src/$package_name/launch/${file_name}_launch.py
        mkdir -p ~/ros2_ws/src/$package_name/launch
        cat << EOF > $launch_file
        from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='$package_name',
            executable='$file_name',
            name='$file_name',
            output='screen'
        ),
    ])
EOF
        echo "Launch file created at $launch_file"
    fi
    ;;
  4)
    # Listener Template
    if [ "$language" == "py" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class $file_name(Node):

    def __init__(self):
        super().__init__('$file_name')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = $file_name()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
        setup_file =~/ros2_ws/src/$package_name/setup.py
        if ! grep -q "$file_name" $setup_file; then
            sed -i "/console_scripts/a \  '$file_name = $package_name.$file_name:main'," $setup_file
        fi

    elif [ "$language" == "cpp" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class $file_name : public rclcpp::Node
{
public:
    $file_name() : Node("$file_name")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&${file_name}::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<$file_name>());
    rclcpp::shutdown();
    return 0;
}
EOF

    cmake_file=~/ros2_ws/src/$package_name/CMakeLists.txt
    if ! grep -q "$file_name" $cmake_file; then
        echo "add_executable($file_name src/file_name.cpp)" >> $cmake_file
        echo "ament_target_dependencies($file_name rclcpp)" >> $cmake_file
        echo "install(TARGETS $file_name DESTINATION lib/\${PROJECT_NAME})" >> $cmake_file
        fi
    fi

    package_file=~/ros2_ws/src/$package_name/package.xml
    if ! grep -q "<exec_depend>example_interfaces</exec_depend>" $package_file; then
        sed -i "/<\/build_depend>/a \ <exec_depend>example_interfaces</exec_depend>" $package_file
    fi

    if [ "$create_launch" == "y" ]; then
        launch_file=~/ros2_ws/src/$package_name/launch/${file_name}_launch.py
        mkdir -p ~/ros2_ws/src/$package_name/launch
        cat << EOF > $launch_file
        from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='$package_name',
            executable='$file_name',
            name='$file_name',
            output='screen'
        ),
    ])
EOF
        echo "Launch file created at $launch_file"
    fi
    ;;
  5)
    # Publisher Template
    if [ "$language" == "py" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class $file_name(Node):

    def __init__(self):
        super().__init__('$file_name')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = $file_name()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
        setup_file =~/ros2_ws/src/$package_name/setup.py
        if ! grep -q "$file_name" $setup_file; then
            sed -i "/console_scripts/a \  '$file_name = $package_name.$file_name:main'," $setup_file
        fi

    elif [ "$language" == "cpp" ]; then
        NODE_FILE="$SRC_PATH/$file_name.$extension"
        cat << EOF > $NODE_FILE
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class $file_name : public rclcpp::Node
{
public:
    $file_name() : Node("$file_name")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&${file_name}::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<$file_name>());
    rclcpp::shutdown();
    return 0;
}
EOF

    cmake_file=~/ros2_ws/src/$package_name/CMakeLists.txt
    if ! grep -q "$file_name" $cmake_file; then
        echo "add_executable($file_name src/file_name.cpp)" >> $cmake_file
        echo "ament_target_dependencies($file_name rclcpp)" >> $cmake_file
        echo "install(TARGETS $file_name DESTINATION lib/\${PROJECT_NAME})" >> $cmake_file
        fi
    fi

    package_file=~/ros2_ws/src/$package_name/package.xml
    if ! grep -q "<exec_depend>example_interfaces</exec_depend>" $package_file; then
        sed -i "/<\/build_depend>/a \ <exec_depend>example_interfaces</exec_depend>" $package_file
    fi

    if [ "$create_launch" == "y" ]; then
        launch_file=~/ros2_ws/src/$package_name/launch/${file_name}_launch.py
        mkdir -p ~/ros2_ws/src/$package_name/launch
        cat << EOF > $launch_file
        from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='$package_name',
            executable='$file_name',
            name='$file_name',
            output='screen'
        ),
    ])
EOF
        echo "Launch file created at $launch_file"
    fi
    ;;
esac

echo "$choice created at $file_path"

echo "Building workspace"
cd ~/ros2_ws
colcon build

echo "Sourcing Workspace"
source install/local_setup.bash

echo "Workspace built and sourced"