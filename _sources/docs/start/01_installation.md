(ros:install)=
# [安装](https://docs.ros.org/en/galactic/Installation.html#)

目前为以下平台提供 ROS 2 二进制包：

*   Ubuntu Linux - Focal Fossa (20.04)

    *   [Debian packages](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

    *   ["fat" archive](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html)

*   RHEL 8

    *   [RPM packages](https://docs.ros.org/en/galactic/Installation/RHEL-Install-RPMs.html)

    *   ["fat" archive](https://docs.ros.org/en/galactic/Installation/RHEL-Install-Binary.html)

*   [Windows](https://docs.ros.org/en/galactic/Installation/Windows-Install-Binary.html)

## 从源代码构建

我们支持在以下平台上从源头构建 ROS 2：

*   [Ubuntu Linux](https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html)
*   [macOS](https://docs.ros.org/en/galactic/Installation/macOS-Development-Setup.html)
*   [RHEL](https://docs.ros.org/en/galactic/Installation/RHEL-Development-Setup.html)
*   [Windows](https://docs.ros.org/en/galactic/Installation/Windows-Development-Setup.html)

## 应该选择哪种安装?

从二进制包或源代码安装都会得到一个功能完整且可用的 ROS 2 安装。不同的选择取决于你计划如何处理 ROS 2。

**二进制包**是通用的，并提供一个已经构建的 ROS 2 安装。这对于那些想要立即开始使用 ROS 2 的人来说是非常棒的。

**从源代码构建**意味着开发者想要改变或明确省略部分 ROS 2 的基础。对于不支持二进制文件的平台，也推荐使用它。从源代码构建还可以让你选择安装绝对最新版本的 ROS 2。

## 在 Ubuntu Linux 上构建 ROS 2

### 系统需求

Galactic Geochelone 目前基于 debian 的目标平台是：

- Tier 1: Ubuntu Linux - Focal Fossa (20.04) 64-bit
- Tier 3: Debian Linux - Bullseye (11) 64-bit

其他支持级别不同的 Linux 平台包括：

*   Arch Linux, see [alternate instructions](https://wiki.archlinux.org/index.php/ROS#ROS_2)

*   Fedora Linux, see [alternate instructions](https://docs.ros.org/en/galactic/Installation/Fedora-Development-Setup.html)

*   OpenEmbedded / webOS OSE, see [alternate instructions](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions)

As defined in [REP 2000](https://www.ros.org/reps/rep-2000.html).

### 系统设置

#### 设置语言环境

确保您有一个支持 `UTF-8` 的语言环境（locale）。如果您在一个最小的环境中（例如 docker 容器），locale 设置可能是 `POSIX` 这样的最小环境。我们使用以下设置进行测试。但是，如果您使用的是支持 UTF-8 的不同语言环境，那么这应该没问题。

```sh
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### 添加 ROS 2 apt 存储库

您需要将 ROS 2 apt 存储库添加到系统中。首先，通过检查这个命令的输出，确保 [Ubuntu Universe](https://help.ubuntu.com/community/Repositories/Ubuntu) 存储库是启用的。

```sh
apt-cache policy | grep universe
```

输出：

```sh
500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
     release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64
```

如果您没有看到如上所示的输出行，那么使用以下指令启用 Universe 存储库。

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

现在将 ROS 2 apt 存储库添加到系统中。首先用 apt 授权我们的 GPG 密钥。

```sh
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

如果 `raw.githubusercontent.com` 报错，可以尝试：

```sh
sudo curl -sSL https://raw.staticdn.net/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

然后将存储库添加到源代码列表中。

```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 安装开发工具和 ROS 工具

```sh
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools
```

Ubuntu 18.04 不是官方支持的平台，但仍然可以工作。你至少需要以下额外的依赖项：

```sh
python3 -m pip install -U importlib-metadata importlib-resources
```

### 获得 ROS 2 代码

创建一个工作区并克隆所有 repo：

```sh
mkdir -p ~/ros2_galactic/src
cd ~/ros2_galactic
# wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos
wget https://raw.staticdn.net/ros2/ros2/galactic/ros2.repos
vcs import src < ros2.repos
```

### 使用 rosdep 安装依赖项

```sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
```

### 安装其他 DDS 实现（可选）

如果你想使用默认的 Eclipse Cyclone DDS 之外的其他 DDS 或 RTPS 供应商，你可以在[这里](https://docs.ros.org/en/galactic/Installation/DDS-Implementations.html)找到说明。

### 在工作区中构建代码

如果您已经通过其他方式安装了 ROS 2（通过debian或二进制发行版），请确保在一个没有其他安装源的新环境中运行以下命令。还要确保你没有 `source /opt/ros/${ROS_DISTRO}/setup.bash` 用你的 `.bashrc`。您可以使用 `printenv | grep -i ROS` 命令确保 ROS 2 没有来源。输出应该为空。

## [通过 Debian Packages 安装 ROS 2](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#id4)

针对 ROS 2 Galactic Geochelone 的 Debian 软件包目前可以在 Ubuntu Focal 上使用。目标平台在 [REP 2000](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst) 中定义。

### 安装 ROS 2 包

在设置存储库之后更新 apt 存储库缓存。

```sh
sudo apt update
```

桌面安装（推荐）：ROS，RViz，demos，tutorials。

```sh
sudo apt install ros-galactic-desktop
```

ROS-Base 安装（Bare Bones）：通信库，消息包，命令行工具。没有 GUI 工具。

```sh
sudo apt install ros-galactic-ros-base
```

### 环境设置

#### 获取安装脚本

通过获取以下文件来设置环境。

```sh
source /opt/ros/galactic/setup.bash
```

### 尝试一些例子

如果你安装了上面的 `ros-galactic-desktop`，你可以尝试一些例子。

在一个终端中，获取安装文件的源代码，然后运行一个 C++ `talker`：

```sh
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_cpp talker
```

在另一个终端源代码安装文件，然后运行 Python `listener`：

```sh
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_py listener
```

你应该看到 `talker` 说它在 `Publishing` 消息，而 `listener` 说我`I heard` 那些消息。这将验证 C++ 和 Python api 是否正常工作。

### 安装后的后续步骤

继续使用[教程和演示](https://docs.ros.org/en/galactic/Tutorials.html)来配置您的环境，创建您自己的工作空间和包，并学习 ROS 2 的核心概念。

### 使用 ROS1 桥接

ROS 1 桥可以连接从 ROS 1 到 ROS 2 的主题，反之亦然。请参阅有关如何构建和使用 ROS1 桥接器的专门[文档](https://github.com/ros2/ros1_bridge/blob/master/README.md)。

### 其他 RMW 实现（可选）

ROS 2 使用的默认中间件是 `Cyclone DDS`，但是该中间件（RMW）可以在运行时被替换。请参阅关于如何使用多个 RMW 的[指南](https://docs.ros.org/en/galactic/How-To-Guides/Working-with-multiple-RMW-implementations.html)。

## 卸载

如果您需要卸载 ROS 2 或切换到基于源代码的安装，一旦您已经从二进制文件安装，运行以下命令：

```sh
sudo apt remove ~nros-galactic-* && sudo apt autoremove
```

## 其他

```{seealso}
[ROS2 基础 - 简书 (jianshu.com)](https://www.jianshu.com/p/5d63c0c93f56)
```