# 安装

## 使用 Debian 软件包

在支持 Debian 包的平台上，最好使用它们，因为它们将与其他系统包一起使用 `apt` 进行更新。

### 在 ROS 项目的背景下

[ROS 项目](https://www.ros.org/) 在其 `apt` 存储库中托管 Debian 包的副本。您可以选择以下两个 `apt` 存储库中的任何一个。

[ROS 2 repository](https://github.com/ros2/ros2/wiki/Linux-Install-Debians#setup-sources)

```sh
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

之后，你可以安装依赖于 `colcon-core` 的 Debian 包以及常用的扩展包（参见 [setup.cfg](https://github.com/colcon/colcon-common-extensions/blob/master/setup.cfg)）。

```sh
$ sudo apt update
$ sudo apt install python3-colcon-common-extensions
```

### ROS 项目外

Debian 包也托管在 [packagecloud](https://packagecloud.io/) 提供的 `apt` 存储库中：

您可以使用以下命令（将在[这里](https://packagecloud.io/dirk-thomas/colcon/install)描述）添加 GPG 密钥和 `apt` 存储库。

```sh
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
```

之后，你可以安装依赖于 `colcon-core` 的 Debian 包以及常用的扩展包（参见 [setup.cfg](https://github.com/colcon/colcon-common-extensions/blob/master/setup.cfg)）。

```sh
sudo apt install python3-colcon-common-extensions
```

## 在任何平台上使用 `pip`

在所有非 Debian 平台上，最常见的安装方式是 Python 包管理器 `pip`。以下假设您使用的是Python 3.5 或更高版本的虚拟环境。如果您想全局安装包，可能需要调用 pip3 而不是 pip，并且需要 `sudo`。

```sh
pip install -U colcon-common-extensions
```

```{note}
包 `colcon-common-extensions` 本身不包含任何功能，只依赖于一组其他包（参见 [setup.cfg](https://github.com/colcon/colcon-common-extensions/blob/master/setup.cfg)）。

您可以使用关键字 `colcon` 在 PyPI 上找到已发布包的列表。
```

## 通过源码包安装

```{note}
这种方法通常只被高级用户使用。
```

通常情况下，当您想尝试或利用已经提交但尚未在发布版本中可用的新特性或 bug 修复时，就会出现这种情况。为了使用上述任何包的最新状态，你可以使用 GitHub 存储库的 URL 调用 `pip`：

```sh
pip install -U git+https://github.com/colcon/colcon-common-extensions.git
```

## 从源代码安装自定义分支

要尝试在拉请求中提出的补丁，你可以通过将分支名称附加到URL来安装该特定分支的源代码：

```sh
pip install -U git+https://github.com/colcon/colcon-core.git@branch_name
```

```{note}
请确保使用 `pip uninstall <name>` 再次卸载自定义版本，以恢复到以前使用的版本。否则，如果你使用 Debian 软件包，这个 `pip` 安装的软件包将覆盖更新的 Debian 软件包。
```