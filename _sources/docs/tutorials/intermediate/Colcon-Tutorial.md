(ros:Colcon-Tutorial)=
# [使用 `colcon` 构建包](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)

```{note}
这是一个简单的教程，如何使用  `colcon` 创建一个 ROS 2 工作区。这是一个实用的教程，而不是设计来取代核心文档。
```

## 背景

`colcon` 是对 ROS 构建工具 `catkin_make`、`catkin_make_isolate`、`catkin_tools` 和 `ament_tools` 的迭代。有关 `colcon` 设计的更多信息，请参阅[此文档](https://design.ros2.org/articles/build_tool.html)。

源代码可以在 [colcon GitHub](https://github.com/colcon) 组织中找到。

## 先决条件

### 安装 `colcon`

````{tab} Linux
```sh
sudo apt install python3-colcon-common-extensions
```
````

````{tab} macOS
```sh
python3 -m pip install colcon-common-extensions
```
````

````{tab} Windows
```sh
pip install -U colcon-common-extensions
```
````

### 安装 ROS 2

为了构建样本，您将需要安装 ROS 2。

按照[安装说明操作](https://docs.ros.org/en/galactic/Installation.html)。

```{attention}
如果使用 Debian 软件包进行安装，本教程要求进行[桌面安装](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#linux-install-debians-install-ros-2-packages)。
```

## 基础

ROS 工作区是具有特定结构的目录。通常有一个 `src` 子目录。在这个子目录中是存放 ROS 包源代码的地方。通常情况下，目录启动时为空。

`colcon` 是基于源代码构建的。默认情况下，它将创建以下目录作为 `src` 目录的对等体：

- `build` 目录将是存储中间文件的地方。对于每个包，将创建一个子文件夹，例如 CMake 被调用。
- `install` 目录是每个包将要安装到的目录。默认情况下，每个包将被安装到单独的子目录中。
- `log` 目录包含关于每个 `colcon` 调用的各种日志信息。

```{note}
与 `catkin` 相比，没有 `devel` 目录。
```
### 创建一个新工作区

首先，创建一个目录（ros2_example_ws）来包含我们的工作区：

````{tab} Linux/macOS
```sh
mkdir -p ~/ros2_example_ws/src
cd ~/ros2_example_ws
```
````
````{tab} Windows
```sh
md \dev\ros2_example_ws\src
cd \dev\ros2_example_ws
```
````

此时，工作区包含一个空目录 `src`：

```sh
.
└── src

1 directory, 0 files
```

### 添加一些源文件

让我们将示例库克隆到工作区的 `src` 目录中：

```sh
git clone https://github.com.cnpmjs.org/ros2/examples src/examples -b galactic
```

现在工作区应该有 ROS 2 示例的源代码：

```sh
.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
```

### 激活覆盖层

重要的是，我们已经为现有的 ROS 2 安装提供了环境，它将为示例包提供必要的构建依赖。这是通过获取二进制安装或源安装提供的安装脚本来实现的。另一个 `colcon` 工作区（请参阅[安装](https://docs.ros.org/en/galactic/Installation.html)）。我们称这种环境为底层。

我们的工作区，`ros2_examples_ws`，将覆盖在现有的 ROS 2 安装之上。通常，当您计划在少量包上进行迭代时，建议使用覆盖，而不是将所有包放到同一个工作区中。

### 构建工作区

```{attention}
要在 Windows 上构建包，你需要在 Visual Studio 环境中，参见构建 [ROS 2 代码](https://docs.ros.org/en/galactic/Installation/Windows-Development-Setup.html#windows-dev-build-ros2) 了解更多细节。
```

在工作区的根目录下，运行 `colcon build`。因为像 `ament_cmake` 这样的构建类型不支持 `devel` 空间的概念，并且需要安装包，所以 `colcon` 支持 `--symlink-install` 选项。这允许通过更改 `source` 空间中的文件（例如 Python 文件或其他未编译资源）来更改已安装的文件，以更快地迭代。

````{tab} Linux/macOS
```sh
colcon build --symlink-install
```
````

````{tab} Windows
```sh
colcon build --symlink-install --merge-install
```
Windows 不允许长路径，所以 `merge-install` 将把所有路径合并到安装目录中。
````

在构建完成后，我们应该看到 `build`、`install` 和 `log` 目录：

```sh
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```

### 运行测试

要对我们刚刚构建的包运行测试，请运行以下命令：

````{tab} Linux/macOS
```sh
colcon test
```
````

````{tab} Windows
记住使用 `x64 Native Tools Command Prompt for VS 2019` 来执行以下命令，因为我们将构建一个工作区。
```sh
colcon test --merge-install
```

您还需要在这里指定 `--merge-install`，因为我们在上面构建时使用了它。
````

### 激活环境

当 `colcon` 成功地完成构建时，输出将位于 `install` 目录中。在使用任何已安装的可执行文件或库之前，您需要将它们添加到您的路径和库路径中。colcon 将在 `install` 目录中生成 `bash/bat` 文件，以帮助设置环境。这些文件将向您的路径和库路径中添加所有必需的元素，并提供由包导出的 bash 或 shell 命令。

````{tab} Linux/macOS
```sh
. install/setup.bash
```
````

````{tab} Windows
```sh
call install\setup.bat
```
或 Powershell：

```sh
install\setup.ps1
```
````

### 尝试演示

有了环境源，我们可以运行由 `colcon` 构建的可执行文件。让我们运行示例中的订阅服务器节点：

```sh
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

在另一个终端中，让我们运行一个发布者节点（不要忘记获取设置脚本的源代码）：

```sh
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

您应该看到来自发布者和订阅者的消息，消息的数量在递增。

## 创建您自己的包

`colcon` 使用 [REP 149](https://www.ros.org/reps/rep-0149.html) 中定义的 `package.xml` 规范（也支持[格式2](https://www.ros.org/reps/rep-0140.html)）。

`colcon` 支持多种构建类型。推荐的构建类型是 `ament_cmake` 和 `ament_python`。也支持纯 `cmake` 包。

`ament_python` 构建的一个例子是 [ament_index_python 包](https://github.com/ament/ament_index/tree/master/ament_index_python)，其中 `setup.py` 是构建的主要入口点。

像 [demo_nodes_cpp](https://github.com/ros2/demos/tree/master/demo_nodes_cpp) 这样的包使用 `ament_cmake` 构建类型，并使用 CMake 作为构建工具。

为了方便起见，您可以使用工具 `ros2 pkg create` 来基于模板创建一个新的包。

```{note}
对于 `catkin` 用户，这相当于 `catkin_create_package`。
```

## 小贴士

- 如果您不想构建一个特定的包，那么在目录中放置一个名为 `COLCON_IGNORE` 的空文件，它将不会被索引。
- 如果你想避免在 CMake 包中配置和构建测试，你可以通过：`--cmake-args -DBUILD_TESTING=0`。
- 如果你想从一个包中运行一个特定的测试：

    ```sh
    colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
    ```