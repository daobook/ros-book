(ros:Writing-A-Simple-Cpp-Publisher-And-Subscriber)=
# [编写一个简单的发布者和订阅者（C++）](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

使用 C++ 创建并运行发布者和订阅者节点。

## 背景

[节点](ros:Understanding-ROS2-Nodes) 是通过 ROS graph 进行通信的可执行进程。在本教程中，节点将通过主题以字符串消息的形式相互传递信息。这里使用的例子是一个简单的“talker”和“listener”系统；一个节点发布数据，另一个节点订阅主题，以便接收该数据。

这些示例中使用的代码可以在[rclcpp topics](https://github.com/ros2/examples/tree/master/rclcpp/topics)找到。

## 先决条件

在前面的教程中，您学习了如何[创建工作区](ros:Creating-A-Workspace)和[创建包](ros:Creating-Your-First-ROS2-Package)。

## 创建一个包

打开一个新的终端并[获取 ROS2 安装的源代码](ros:Configuring-ROS2-Environment)，这样 `ros2` 命令就可以工作了。

导航到[上一教程](ros:Creating-A-Workspace)中创建的 `dev_ws` 目录。

回想一下，包应该在 `src/` 目录中创建，而不是在工作区的根目录中。因此，导航到 `dev_ws/src`，并运行包创建命令：

```sh
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

终端将返回一条消息，验证 `cpp_pubsub` 包及其所有必要文件和文件夹的创建情况。

导航到 `dev_ws/src/cpp_pubsub/src`。回想一下，这是任何 CMake 包中包含可执行文件的源文件所在的目录。

## 编写 publisher 节点

输入以下命令下载示例 talker 代码：

````{tab} Linux/macOS
```sh
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp
```
````

````{tab} Windows
在 Windows 命令行提示符中：

```sh
curl -sk https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp -o publisher_member_function.cpp
```

或者在 powershell：

```sh
curl https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp -o publisher_member_function.cpp
```
````

现在将有一个名为 `publisher_member_function.cpp` 的新文件。使用首选文本编辑器打开文件。

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### 检查代码

代码的顶部包含您将要使用的标准 C++ 头文件。在标准的 C++ 头文件之后是 `rclcpp/rclcpp.hpp`，它允许你使用 ROS 2 系统中最常见的部分。最后是 `std_msgs/msg/string.hpp`，其中包含用于发布数据的内置消息类型。

这些行表示节点的依赖项。还记得吗，必须将依赖项添加到 `package.xml` 和 `CMakeLists.txt` 中，这将在下一节中完成。

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```

下一行通过继承 `rclcpp::Node` 来创建节点类 `MinimalPublisher`。代码中的每一个 `this` 都指向节点。

```cpp
class MinimalPublisher : public rclcpp::Node
```

公共构造函数将节点命名为 `minimal_publisher`，并将 `count_` 初始化为 `0`。在构造函数内部，发布者被初始化为 `String` 消息类型、主题名称 `topic` 以及在发生备份时限制消息所需的队列大小。接下来，初始化 `timer_`，这将导致每秒执行两次 `timer_callback` 函数。

```cpp
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
```

`timer_callback` 函数是设置消息数据和实际发布消息的地方。`RCLCPP_INFO` 宏确保将每个发布的消息打印到控制台。

```cpp
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
```

最后是定时器、发布者和计数器字段的声明。

```cpp
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
```

紧随 `MinimalPublishe` 类之后的是 `main`，节点实际执行的位置。`rclcpp::init` 初始化 ROS 2，`rclcpp::spin` 开始处理节点数据，包括定时器的回调。

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### 添加依赖项

导航一层回到 `dev_ws/src/cpp_pubsub` 目录，这里已经为您创建了 `CMakeLists.txt` 和 `package.xml` 文件。

用文本编辑器打开 `package.xml`。

正如之前的教程中提到的，确保填写 `<description>`， `<maintainer>` 和 `<license>` 标签：

```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

在 `ament_cmake` 的 `buildtool` 依赖项后面添加一行，并将下面的依赖项粘贴到你的节点的 `include` 语句中：

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

声明包在执行代码时需要 `rclcpp` 和 `std_msgs`。

确保保存该文件。

### CMakeLists.txt

现在打开 `CMakeLists.txt` 文件。在现有依赖项 `find_package(ament_cmake REQUIRED)` 下面添加以下行：

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

之后，添加可执行文件，并将其命名为 `talker`，这样您就可以使用 `ros2 run` 运行您的节点：

```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

最后，添加 `install(TARGETS…)` 部分，这样 `ros2 run` 就可以找到你的可执行文件：

```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

你可以清理你的 `CMakeLists.txt` 通过删除一些不必要的部分和注释，所以它看起来像这样：

```
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

您现在可以构建您的包，获取本地安装文件的源代码，并运行它，但是让我们先创建订阅者节点，这样您就可以看到整个系统在工作。

## 编写订阅节点

返回到 `dev_ws/src/cpp_pubsub/src` 以创建下一个节点。在终端中输入以下代码：

````{tab} Linux/macOS
```sh
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
```
````

````{tab} Windows
在 Windows 命令行提示符中：

```sh
curl -sk https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp -o subscriber_member_function.cpp
```

或者在 powershell：

```sh
curl https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp -o subscriber_member_function.cpp
```
````

在控制台中输入 `ls` 将会返回：

```sh
publisher_member_function.cpp  subscriber_member_function.cpp
```

使用文本编辑器打开 `subscriber_member_function.cpp`。

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### 检查代码

订阅者节点的代码与发布者节点的代码几乎相同。现在节点被命名为 `minimal_subscriber`，构造函数使用节点的 `create_subscription` 类来执行回调。

没有计时器，因为只要数据发布到 `topic` 主题，订阅者就会响应。

```cpp
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
```

回忆一下 [主题教程](ros:Understanding-ROS2-Topics) 中的内容，发布者和订阅者使用的主题名称和消息类型必须匹配才能进行通信。

`topic_callback` 函数接收在主题上发布的字符串消息数据，并使用 `RCLCPP_INFO` 宏将其写入控制台。

该类中唯一的字段声明是订阅。

```cpp
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

`main` 函数完全相同，只是现在它旋转 `MinimalSubscriber` 节点。对于发布者节点，旋转意味着启动计时器，但对于订阅者，这仅仅意味着准备在消息到来时接收消息。

由于该节点与 `publisher` 节点具有相同的依赖项，所以 `package.xml` 中没有新的内容可以添加。

### CMakeLists.txt

重新打开 `CMakeLists.txt`，并在发布者条目下面添加订阅者节点的可执行文件和目标。

```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

确保保存文件，然后您的 pub/sub 系统就可以使用了。

## 构建并运行

您可能已经安装了 `rclcpp` 和 `std_msgs` 包作为您的 ROS 2 系统的一部分。在你的工作空间的根目录（`dev_ws`）中运行 `rosdep` 是一个很好的实践，在构建之前检查缺失的依赖项：

````{tab} Linux
```sh
rosdep install -i --from-path src --rosdistro galactic -y
```
````

````{tab} macOS/Windows
`rosdep` 只能在 Linux 上运行，所以您可以跳到下一步。
````

仍然在你的工作空间的根，`dev_ws`，构建你的新包：

````{tab} Linux/macOS
```sh
colcon build --packages-select cpp_pubsub
```
````

````{tab} Windows
```sh
colcon build --merge-install --packages-select cpp_pubsub
```
````

打开一个新的终端，导航到 `dev_ws`，并获取安装文件的源代码：

````{tab} Linux/macOS
```sh
. install/setup.bash
```
````

````{tab} Windows
```sh
call install/setup.bat
```
````

现在运行 `talker` 节点：

```sh
ros2 run cpp_pubsub talker
```

终端应该每 0.5 秒发布一次信息消息，像这样：

```sh
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

打开另一个终端，再次从 `dev_ws` 内部获取安装文件，然后启动 listener 节点：

```sh
ros2 run cpp_pubsub listener
```

listener 将开始将消息打印到控制台，从发布者当时的消息计数开始，如下所示：

```sh
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

在每个终端中输入 {kbd}`Ctrl`+{kbd}`C`，停止节点旋转。

## 小结

您创建了两个节点来通过主题发布和订阅数据。在编译和运行它们之前，您将它们的依赖项和可执行文件添加到包配置文件中。

## 下一步

接下来，您将使用服务/客户端模型创建另一个简单的 ROS 2 包。同样，您可以选择用 [C++](ros:Writing-A-Simple-Cpp-Service-And-Client) 或 [Python](ros:Writing-A-Simple-Py-Service-And-Client) 编写它。

## 相关内容

有几种方法可以在 C++ 中编写发布者和订阅者；请查看 [ros2/examples](https://github.com/ros2/examples/tree/master/rclcpp/topics) 仓库中的 `minimal_publisher` 和 `minimal_subscriber` 包。