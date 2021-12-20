# VSCode ROS2 Workspace Template

参考 [athackst/vscode_ros2_workspace: A template for using VSCode as an IDE for ROS2 development. (github.com)](https://github.com/athackst/vscode_ros2_workspace)。

此模板将帮助您使用 ROS2 和 VSCode 作为 IDE 进行设置。

阅读[如何用 VSCode 和 ROS2 开发](https://www.allisonthackston.com/articles/vscode_docker_ros2.html)，以更深入地了解如何使用这个工作区。

## 特性

### 风格

ROS2 批准的格式化程序包括在 IDE 中。
- C++ uncrustify：配置 `ament_uncrustify` 
- Python autopep8：与[样式指南](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)一致的 VSCode 设置

### Tasks

有许多预定义的任务，见 [`.vscode/tasks.json`](https://github.com/athackst/vscode_ros2_workspace/blob/foxy/.vscode/tasks.json) 的完整清单。请随意调整它们以满足您的需要。

阅读[如何使用任务进行开发的](https://www.allisonthackston.com/articles/vscode_tasks.html)，以了解我如何在开发中使用任务。

### 调试

这个模板设置了 Python 文件的调试和 cpp 程序的 gdb。阅读 [`.vscode/launch.json`](https://github.com/athackst/vscode_ros2_workspace/blob/foxy/.vscode/launch.json) 配置细节。

### 持续集成

该模板还附带了基本的持续集成设置。看 [`.github/workflows/ros.yaml`](https://github.com/athackst/vscode_ros2_workspace/blob/foxy/.github/workflows/ros.yaml)。

要删除 `linter`，只需从这一行中删除它的名称：

```yaml
      matrix:
          linter: [cppcheck, cpplint, uncrustify, lint_cmake, xmllint, flake8, pep257]
```

## 如何使用该模板

### 先决条件

你应该已经在你的系统上安装了：

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
