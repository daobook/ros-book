# CLI 工具

> {sub-ref}`today` | 阅读时长 {sub-ref}`wordcount-minutes` 分

(ros:configuring-ROS2-environment)=
## [配置 ROS 2 环境](https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html)

### 背景

ROS 2 依赖于使用 shell 环境组合工作空间（workspaces）的概念。“**工作区**”（workspaces）是一个 ROS 术语，指的是你使用 ROS 2 进行开发的系统上的位置。核心 ROS 2 工作空间称为**底层**（underlay）。随后的本地工作区称为**覆盖**（overlays）。在使用 ROS 2 进行开发时，通常会同时有几个工作区处于活动状态。

组合工作空间使得针对不同版本的 ROS 2 或针对不同的包集进行开发更加容易。它还允许在同一台计算机上安装多个 ROS 2 发行版（或 “distros”，例如 Dashing 和 Eloquent）并在它们之间进行切换。

这可以通过每次打开新的 shell 时都提供安装文件，或者将 source 命令添加到 shell 启动脚本一次来实现。没有来源的安装文件，你将无法访问 ROS 2 命令，或找到或使用 ROS 2 包。换句话说，你不能使用 ROS 2。

### 先决条件

在开始这些教程之前，按照 [ROS 2 安装](https://docs.ros.org/en/galactic/Installation.html#installationguide) 页面上的说明安装 ROS 2。

本教程中使用的命令假设您遵循了操作系统的二进制包安装指南（Linux 的 Debian 包）。如果您是从源代码构建的，您仍然可以按照下面的方法进行操作，但是安装文件的路径可能会有所不同。如果你从源代码安装，你也不能使用 `sudo apt install ros-<distro>-<package>` 命令（在新手教程中经常使用）。

如果您正在使用 Linux 或 macOS，但不熟悉 shell，[Unix 教程](http://www.ee.surrey.ac.uk/Teaching/Unix/) 将有所帮助。

### 任务

:::{panels}
:container: +full-width text-center
:header: w3-pale-blue w3-wide w3-card-4 
:column: col-lg-12 px-2 py-2
:body: text-justify w3-padding
:card: w3-pale-red w3-card
---
1 Source 安装文件
^^^
你需要在你打开的每一个 shell 上运行这个命令来访问 ROS 2 命令，就像这样：

````{tab} Linux
```console
source /opt/ros/{DISTRO}/setup.bash
```
````

````{tab} Windows
```console
call C:\dev\ros2\local_setup.bat
```
````

````{tab} macOS
```console
. ~/ros2_install/ros2-osx/setup.bash
```
````

```{note}
确切的命令取决于您安装 ROS 2 的位置。如果遇到问题，请确保文件路径指向您的安装。
```
---
2 在 shell 启动脚本中添加“source”
^^^
如果你不想每次打开一个新的shell（跳过任务 1）时都要获取安装文件，那么你可以将这个命令添加到 shell 启动脚本中：

````{tab} Linux
```console
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
```
要撤消此命令，请找到系统的 shell 启动脚本并删除附加的 source 命令。
````

````{tab} Windows
仅对 PowerShell 用户，在“我的文档”中创建一个名为“WindowsPowerShell”的文件夹。在'WindowsPowerShell'中，创建文件'Microsoft.PowerShell_profile.ps1'。在文件内部，粘贴：
```powershell
C:\dev\ros2_galactic\local_setup.ps1
```
每次打开一个新的 shell 时，PowerShell 都会请求运行此脚本的权限。为了避免这个问题，你可以运行：
```powershell
Unblock-File C:\dev\ros2_galactic\local_setup.ps1
```
要撤消此操作，请删除新的 ‘Microsoft.PowerShell_profile.ps1’ 的文件。
````

````{tab} macOS
```console
echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile
```
要撤消此命令，请找到系统的 shell 启动脚本并删除附加的 source命令。
````
---
3 将 `colcon_cd` 添加到 shell 启动脚本中
^^^
`colcon_cd` 命令允许您快速地将 shell 的当前工作目录更改为包的目录。例如，`colcon_cd some_ros_package` 会很快将您带到 `~/ros2_install/src/some_ros_package` 目录。


````{tab} Linux
```console
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
```
````

````{tab} Windows
没有可用的
````

````{tab} macOS
```console
TODO
```
````

根据您安装 `colcon_cd` 的方式和您的工作空间所在的位置，上面的说明可能会有所不同，请参阅[文档](https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes)了解更多细节。要在 Linux 和 macOS 中撤消此操作，请定位系统的 shell 启动脚本并删除附加的源和导出命令。
---
4 检查环境变量
^^^
获取 ROS 2 安装文件将设置操作 ROS 2 所需的几个环境变量。如果你在寻找或使用你的 ROS 2 包时遇到问题，请确保使用以下命令正确设置你的环境：


````{tab} Linux
```console
printenv | grep -i ROS
```
````

````{tab} Windows
```console
set | findstr -i ROS
```
````

````{tab} macOS
```console
printenv | grep -i ROS
```
````

检查是否设置了 `ROS_DISTRO` 和 `ROS_VERSION` 等变量。

```console
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=galactic
```

如果环境变量设置不正确，请返回安装指南中的安装 ROS 2 包章节。如果您需要更具体的帮助（因为环境设置文件可能来自不同的地方），您可以从社区[获得答案](https://answers.ros.org/)。
:::

#### `ROS_DOMAIN_ID` 变量

有关 ROS 域 ID 的详细信息，请参阅 [domain ID](https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html) 文章。

一旦您为您的 ROS 2 代理组确定了一个唯一的整数，您可以使用以下命令设置环境变量：

````{tab} Linux
```console
export ROS_DOMAIN_ID=<your_domain_id>
```
为了在 shell 会话之间保持这个设置，你可以在 shell 启动脚本中添加这个命令：

```console
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
````

````{tab} Windows
```console
set ROS_DOMAIN_ID=<your_domain_id>
```

如果你想让这个在 shell 会话之间永久存在，也可以运行：

```console
setx ROS_DOMAIN_ID <your_domain_id>
```
````

````{tab} macOS
```console
export ROS_DOMAIN_ID=<your_domain_id>
```
为了在 shell 会话之间保持这个设置，你可以在 shell 启动脚本中添加这个命令：
```console
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile
```
````

### 总结

ROS 2 开发环境在使用前需要进行正确配置。这可以通过两种方式实现：要么在每个打开的新 shell 中获取安装文件，要么将 `source` 命令添加到启动脚本中。

如果您在定位或使用 ros2 包时遇到任何问题，您应该做的第一件事是检查环境变量，并确保它们被设置为您想要的版本和发行版。

(ros:introducing-turtlesim)=
## [介绍 turtlesim 和 rqt](https://docs.ros.org/en/galactic/Tutorials/Turtlesim/Introducing-Turtlesim.html)

Turtlesim 是一个轻量级的学习 ROS 2 的模拟器。它说明了 ROS 2 在最基本的水平上的作用，让你对以后使用真实机器人或机器人模拟做什么有一个概念。

rqt 是 ROS2 的 GUI 工具。在 rqt 中完成的所有操作都可以在命令行上完成，但它提供了一种更简单、更用户友好的方式来操作 ROS 2 元素。

本教程涉及核心 ROS 2 概念，如节点、主题和服务的分离。所有这些概念将在后面的教程中详细阐述；现在，您只需设置这些工具并对它们有一些感觉。

### 先决条件

前面的教程，[](ros:configuring-ROS2-environment)，向您展示如何设置您的环境。

### 任务

:::{panels}
:container: +full-width text-center
:header: w3-pale-blue w3-wide w3-card-4 
:column: col-lg-12 px-2 py-2
:body: text-justify w3-padding
:card: w3-pale-red w3-card
---
1 安装 turtlesim
^^^
像往常一样，首先在一个新的终端中找到安装文件，如 [上一节教程](ros:configuring-ROS2-environment) 中所述。

为您的 ROS 2 发行版安装 turtlesim 包：

````{tab} Linux
```console
sudo apt update

sudo apt install ros-galactic-turtlesim
```
````

````{tab} Windows
只要您从 ROS2 中安装的归档文件包含 `ros_tutorials` 存储库，就应该已经安装了 `turtlesim`。
````

````{tab} macOS
只要您从 ROS2 中安装的归档文件包含 `ros_tutorials` 存储库，就应该已经安装了 `turtlesim`。
````

检查安装包：

```sh
ros2 pkg executables turtlesim
```

上面的命令应该返回 `turtlesim` 的可执行文件列表：

```sh
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```
---
2 启动 `turtlesim`
^^^
要启动 `turtlesim`，在终端输入以下命令：

```sh
ros2 run turtlesim turtlesim_node
```

模拟器窗口将出现，中间随机出现一只海龟。

![](images/turtlesim.png)

在命令下的终端中，您将看到来自该节点的消息：

```sh
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim

[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

这里你可以看到你的默认海龟的名字是 `turtle1`，以及它生成的默认坐标。
---
3 使用 `turtlesim`
^^^
再次打开一个新的终端和 source ROS 2。

现在你要运行一个新节点来控制第一个节点中的 `turtle`：

```sh
ros2 run turtlesim turtle_teleop_key
```

此时，您应该打开了三个窗口：一个运行 `turtlesim_node` 的终端、一个运行 `turtle_teleop_key` 的终端和 `turtlesim` 窗口。安排这些窗口，以便您可以看到 `turtlelesim` 窗口，但也让终端运行 `turtle_teleop_key` 活动，以便您可以在 `turtlelesim` 中控制 turtle。

使用键盘上的方向键来控制海龟。它将在屏幕上移动，使用它所附的“笔”来绘制到目前为止它所遵循的路径。

```{note}
按下箭头键只会使海龟移动一小段距离，然后停止。这是因为，实际上，如果操作员失去了与机器人的连接，你不会希望机器人继续执行指令。
```

您可以使用 `list` 命令查看节点及其相关的服务、主题和操作：

```sh
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

在接下来的教程中，您将学习更多关于这些概念的知识。由于本教程的目标只是对 `turtlesim` 进行总体概述，我们将使用 `rqt`（ROS 2 的图形用户界面）来更近距离地了解服务。
---
4 安装 rqt
^^^
打开一个新的终端安装 `rqt` 和它的插件：

````{tab} Linux（apt 2.0/Ubuntu 20.04 或更新）
```console
sudo apt update

sudo apt install ~nros-galactic-rqt*
```
````

````{tab} Windows/macOS
在 Windows/macOS 上安装 ROS 2 的标准归档文件包含 `rqt` 和它的插件，所以你应该已经安装了 `rqt`。
````

`rqt` 运行：

```sh
rqt
```
---
5 使用 rqt
^^^
第一次运行 `rqt` 后，窗口将为空。不用担心；只需从顶部的菜单栏中选择 `Plugins > Services > Service Caller` 即可。

```{note}
rqt 本身可能需要一些时间来定位所有的插件。如果你点击了 `Plugins`，但是没有看到 `Services` 或任何其他选项，你应该关闭 `rqt`，在终端输入命令 `rqt --force-discover`。
```

![](images/rqt.png)

使用 `Service` 下拉列表左侧的刷新按钮，以确保 `turtlesim` 节点的所有服务都可用。

点击服务下拉列表查看 `turtlesim` 的服务，并选择 `/spawn` 服务。
:::

#### 5.1 尝试刷出服务

让我们使用 `rqt` 来调用 `/spawn` 服务。你可以从它的名字猜到 `/spawn` 将在 turtlesim 窗口中创建另一只海龟。

通过在 `Expression` 列的空单引号之间双击，为新海龟指定一个唯一的名称，如 `turtle2`。您可以看到这个表达式对应于 `name` 值，并且是 `string` 类型的。

输入海龟产卵的新坐标，如 `x = 1.0` 和 `y = 1.0`。

![](images/spawn.png)

````{note}
如果你试图生成一个与现有海龟同名的新海龟，比如默认的 `turtle1`，你会在运行 `turtlesim_node` 的终端中得到一个错误消息：

```sh
[ERROR] [turtlesim]: A turtle named [turtle1] already exists
```
````

要生成 `turtle2`，您必须通过单击 `rqt` 窗口右上角的 `call` 按钮来调用服务。

你会在你输入的 `x` 和 `y` 坐标处看到一只新的海龟（同样是随机设计的）。

如果您刷新 `rqt` 中的服务列表，您还将看到现在除了 `/turtle1/....` 之外，还有与新的 `turtle` 相关的服务，`/turtle2/…`。

#### 5.2 尝试使用 `set_pen` 服务

现在让我们使用 `/set_pen` 服务给 `turtle1` 一个唯一的笔：

![](images/set_pen.png)

`r`, `g` 和 `b` 的值在 0 到 255 之间，将设置画笔 `turtle1` 绘制的颜色，`width` 设置线条的粗细。

要让 `turtle1` 用清晰的红线绘制，请将 `r` 的值更改为 `255`，将 `width` 的值更改为 `5`。不要忘记在更新值之后 call 服务。

如果您返回 `turtle_teleop_node` 运行的终端并按下方向键，您将看到 `turtle1` 的钢笔已经更改。

![](images/new_pen.png)

你可能已经注意到海龟是无法移动的。可以通过将 `turtle1` 的 `cmd_vel` 主题重新映射到 `turtle2` 来实现这一点。

### 6 重新映射

在一个新的终端中，source ROS 2，并运行：

```sh
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

现在，您可以在这个终端激活时移动 `turtle2`，在运行 `turtle_teleop_key` 的另一个终端激活时移动 `turtle1`。

![](images/remap.png)

### 7 关闭 turtlesim

在 `turtlesim_node` 终端输入 `Ctrl + C`，在 `teleop` 终端输入 `q`，可以停止模拟。

(ros:understanding-ROS2-nodes)=
## 理解 ROS2 节点

### 背景

:::{panels}
:container: +full-width text-center
:header: w3-pale-blue w3-wide w3-card-4 
:column: col-lg-12 px-2 py-2
:body: text-justify w3-padding
:card: w3-pale-red w3-card
---
1 ROS 2 graph
^^^
在接下来的教程中，你将学习一系列构成“ROS2 graph”的核心 ROS 2 概念。

ROS graph 是 ROS 2 元素同时处理数据的网络。它包含所有可执行文件以及它们之间的连接，如果您要将它们全部映射出来并可视化的话。
---
2 ROS 2 中的节点
^^^
ROS 中的每个节点应负责单个模块用途（例如，一个节点用于控制轮电机，一个节点用于控制激光测距仪，等等）。每个节点可以通过话题（topics）、服务（services）、动作（actions）或参数（parameters）向其他节点发送和接收数据。

![](images/Nodes-TopicandService.gif)

一个完整的机器人系统由许多协同工作的节点组成。在 ROS 2 中，一个可执行文件（C++程序、Python 程序等）可以包含一个或多个节点。
:::

### 先决条件

[前面的教程](ros:introducing-turtlesim) 向您展示了如何安装这里使用的 `turtlesim` 包。

和往常一样，不要忘记在你打开的每一个[新终端中都输入 ROS 2](https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html#configros2)。

### 任务

:::{panels}
:container: +full-width text-center
:header: w3-pale-blue w3-wide w3-card-4 
:column: col-lg-12 px-2 py-2
:body: text-justify w3-padding
:card: w3-pale-red w3-card
---
1 `ros2 run`
^^^
命令 `ros2 run` 从包中启动一个可执行文件。
```sh
ros2 run <package_name> <executable_name>
```

运行 `turtlesim`，打开一个新的终端，并输入以下命令：

```sh
ros2 run turtlesim turtlesim_node
```

这里，包名是 `turtlesim`，可执行文件名是 `turtlesim_node`。

然而，我们仍然不知道节点名。您可以使用 `ros2 node list` 查找节点名称。
---
2 `ros2 node list`
^^^
`ros2 node list` 将显示所有运行节点的名称。当您希望与某个节点进行交互，或者当您的系统运行多个节点并需要跟踪它们时，这一点特别有用。

当 `turtlesim` 在另一个终端中运行时，打开一个新的终端，并输入以下命令：

```sh
ros2 node list
```

终端将返回节点名：

```sh
/turtlesim
```

打开另一个新的终端并使用以下命令启动 `teleop` 节点：

```sh
ros2 run turtlesim turtle_teleop_key
```

在这里，我们再次搜索 turtlesim 包，这次搜索的是名为 `turtle_teleop_key` 的可执行文件。

返回到运行 `ros2 node list` 的终端，并再次运行它。现在您将看到两个活动节点的名称：

```sh
/turtlesim
/teleop_turtle
```
:::

#### 2.1 重新映射

[重新映射](https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules) 允许您将默认节点属性（如节点名称、主题名称、服务名称等）重新分配给自定义值。在上一篇教程中，您使用 `turtle_teleop_key` 上的重映射来更改正在控制的默认海龟。

现在，让我们重新分配 `/turtlesim` 节点的名称。在新的终端中运行如下命令：

```sh
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

既然你再次调用 `ros2 run`，另一个 turtlesim 窗口将打开。但是，现在如果您返回到运行 `ros2 node list` 的终端，并再次运行它，您将看到三个节点名：

```sh
/my_turtle
/turtlesim
/teleop_turtle
```

### 3 `ros2 node info`

现在您知道了节点的名称，可以使用以下方法访问更多关于节点的信息：

```sh
ros2 node info <node_name>
```

要检查最新的节点 `my_turtle`，运行以下命令：

```sh
ros2 node info /my_turtle
```

`ros2 node info` 返回与该节点交互的订阅者、发布者、服务和操作（ROS graph 连接）的列表。输出应该是这样的：

```sh
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

现在尝试在 `/teleop_turtle` 节点上运行相同的命令，看看它的连接与 `my_turtle` 有何不同。

您将在接下来的教程中了解更多关于 ROS graph 连接的概念，包括消息类型。

### 总结

节点是一个基本的 ROS 2 元素，在机器人系统中为一个单一的、模块化的目的服务。

在本教程中，通过运行可执行文件 `turtlesim_node` 和 `turtle_teleop_key`，您使用了从 `turtlesim` 包中创建的节点。

您了解了如何使用 `ros2 node list` 来发现活动节点名称和 `ros2 node info`，以便在单个节点上进行内省。这些工具对于理解复杂的、真实世界的机器人系统中的数据流至关重要。

(ros:Understanding-ROS2-Topics)=
## 了解 ROS 2 话题

### 背景

ROS 2 将复杂的系统分解成许多模块化节点。话题是 ROS graph 的一个重要元素，它充当节点交换消息的总线。

![](images/Topic-SinglePublisherandSingleSubscriber.gif)

节点可以向任意数量的话题发布数据，并同时订阅任意数量的话题。

![](images/Topic-MultiplePublisherandMultipleSubscriber.gif)

话题是数据在节点之间以及系统的不同部分之间移动的主要方式之一。

### 先决条件

[前面的教程](ros:understanding-ROS2-nodes) 提供了一些关于在此基础上构建的节点的有用背景信息。