(ros:Creating-Your-First-ROS2-Package)=
# [创建你的第一个 ROS 2 包](https://docs.ros.org/en/galactic/Tutorials/Creating-Your-First-ROS2-Package.html)

使用 CMake 或 Python 创建一个新程序包，然后运行其可执行文件。

## 背景

### [什么是 ROS 2 包？](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#id2)[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#what-is-a-ros-2-package "Permalink to this headline")

可以将包视为 ROS 2 代码的容器。如果您希望能够安装代码或与他人共享代码，则需要将其组织在一个软件包中。使用软件包，您可以发布 ROS2 的工作，并允许其他人轻松构建和使用它。

ROS2 中的软件包创建使用 `ament` 作为其构建系统，并使用 `colcon` 作为其构建工具。您可以使用官方支持的 CMake 或 Python 创建软件包，但也存在其他构建类型。

### [ROS 2 包由什么组成？](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#id3)[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#what-makes-up-a-ros-2-package "Permalink to this headline")

ROS 2 Python 和 CMake 软件包各自具有自己的最低要求内容：

:::{tab} CMake
- `package.xml` 文件，其中包含有关程序包的元信息
- `CMakeLists.txt` 文件，描述如何在包中构建代码
:::

:::{tab} Python
- `package.xml` 文件，其中包含有关程序包的元信息
- `setup.py` 包含有关如何安装软件包的说明
- 软件包包含可执行文件时，需要 `setup.cfg`，以便 `ros2 run` 可以找到它们
- `/<package_name>`：ROS 2 工具用来查找软件包的与软件包同名的目录包含 `__init__.py`
:::

最简单的包可能具有如下文件结构：

:::{tab} CMake
```js
my_package/
     CMakeLists.txt
     package.xml
```
:::

:::{tab} Python
```js
my_package/
      setup.py
      package.xml
      resource/my_package
```
:::

### [工作区中的包](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#id4)[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#packages-in-a-workspace "Permalink to this headline")

一个工作空间可以包含所需数量的程序包，每个程序包都位于其自己的文件夹中。您还可以在一个工作空间（CMake，Python等）中拥有不同构建类型的软件包。您不能有嵌套的程序包。

最佳做法是在工作区中有一个 `src` 文件夹，并在其中创建包。这样可以使工作空间的顶层保持“干净”。

典型的一个工作空间内功能包的结构如下：

```sh
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

## 先决条件

在遵循 [上一教程](ros:Creating-A-Workspace) 中的说明之后，您应该有一个 ROS 2 工作区。您将在这个工作区中创建您的包。

## 任务

### [创建一个包](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#id7)[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#create-a-package "Permalink to this headline")

首先，source 您的 [](ros:Configuring-ROS2-Environment)。

让我们为您的新包，使用您在[上一教程](ros:Creating-A-Workspace) 中创建的工作空间 `dev_ws`。

运行软件包创建命令之前，请确保您位于 `src` 文件夹中。

在 ROS 2 中创建新程序包的命令语法为：

````{tab} CMake
```sh
ros2 pkg create --build-type ament_cmake <package_name>
```
````

````{tab} Python

```sh
ros2 pkg create --build-type ament_python <package_name>
```
````

在终端中输入以下命令：

```sh
cd dev_ws/src
```

在本教程中，您将使用可选参数 `--node-name`，它在包中创建一个简单的 Hello World 类型可执行文件。

在终端中输入以下命令：

````{tab} CMake

```sh
ros2 pkg create --build-type ament_cmake --node-name my_node my_package_cpp
```
````

````{tab} Python

```sh
ros2 pkg create --build-type ament_python --node-name my_node my_package_py
```
````

现在，您将在工作区的 `src` 目录中拥有一个名为 `my_package_py` 与 `my_package_cpp` 的新文件夹。运行命令后，您的终端将返回以下消息：

````{tab} CMake

```sh
going to create a new package
package name: my_package
destination directory: /home/user/dev_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source and include folder
creating folder ./my_package/src
creating folder ./my_package/include/my_package
creating ./my_package/CMakeLists.txt
creating ./my_package/src/my_node.cpp
```
````
````{tab} Python

```sh
going to create a new package
package name: my_package
destination directory: /home/user/dev_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source folder
creating folder ./my_package/my_package
creating ./my_package/setup.py
creating ./my_package/setup.cfg
creating folder ./my_package/resource
creating ./my_package/resource/my_package
creating ./my_package/my_package/__init__.py
creating folder ./my_package/test
creating ./my_package/test/test_copyright.py
creating ./my_package/test/test_flake8.py
creating ./my_package/test/test_pep257.py
creating ./my_package/my_package/my_node.py
```
````

您可以看到为新软件包自动生成的文件。

### [构建一个包](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#id8)[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#build-a-package "Permalink to this headline")

将软件包放在工作空间中特别有价值，因为您可以通过在工作空间根目录中运行 `colcon build` 一次构建许多软件包。否则，您将必须分别构建每个软件包。

返回工作区的根目录，现在您可以构建软件包：

:::{tab} Linux/macOS
```sh
cd ..
colcon build
```
:::

:::{tab} Windows
```sh
cd ..
colcon build --merge-install
```
Windows 不允许长路径，所以 `merge-install` 将把所有路径合并到安装目录中。
:::

回想一下上一个教程，在您的 `dev_ws` 中也有 `ros_tutorials` 包。您可能已经注意到，运行 `colcon build` 也构建了 `turtlesim` 包。当您的工作区中只有几个包时，这很好，但是当有很多包时，`colcon` 构建可能会花费很长时间。

要想下次只构建 `my_package` 包，可以运行以下命令：

```sh 
colcon build --packages-select my_package
```

### Source setup 文件[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#source-the-setup-file "Permalink to this headline")

要使用新的程序包和可执行文件，请先打开一个新终端，然后获取 ROS2 的主要安装源。然后，从 `dev_ws` 目录中，运行以下命令来获取您的工作空间：

:::{tab} Linux/macOS
```sh
. install/setup.bash
```
:::

:::{tab} Windows
```sh
call install/local_setup.bat
```
:::

现在您的工作区已添加到路径中，您将可以使用新程序包的可执行文件。

### 使用包[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#use-the-package "Permalink to this headline")

要在软件包创建期间运行使用 `--node-name` 参数创建的可执行文件，请输入以下命令：

```sh
ros2 run my_package my_node
```

它会向你的终端返回一条消息：

:::{tab} CMake
输入：

```sh
ros2 run my_package_cpp my_node
```

输出：

```sh
hello world my_package_cpp package
```
:::

:::{tab} Python

输入：

```sh
ros2 run my_package_py my_node
```

输出：

```sh
Hi from my_package_py.
```
:::

### [检查包装内容](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#id9)[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#examine-package-contents "Permalink to this headline")

在 `dev_ws/src/my_package` 中，你会看到 `ros2 pkg create` 的文件和文件夹自动生成：

:::{tab} CMake
```sh
CMakeLists.txt  include  package.xml  src
```
`my_node.cpp` 位于 `src` 目录中。这是您所有的自定义 C++ 节点在未来的位置。
:::

:::{tab} Python
```sh
my_package  package.xml  resource  setup.cfg  setup.py  test
```
`my_node.py` 位于 `my_package` 目录中。这是您的所有自定义 Python 节点在未来的位置。
:::

### 定制 `package.xml`[¶](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#customize-package-xml "Permalink to this headline")

创建软件包后，`package.xml` 文件需要我们手动完善一下。您可能在返回消息中注意到字段 `description` 和 `license` 包含 `TODO` 注释。这是因为软件包 `description` 和 `license` 声明不会自动设置，但是如果您要释放软件包，则必须设置。`maintainer` 字段可能也需要填写。

从 `dev_ws/src/my_package`，使用你喜欢的文本编辑器打开 `package.xml`：

:::{tab} CMake
```xml
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <buildtool_depend>ament_cmake</buildtool_depend>

 <test_depend>ament_lint_auto</test_depend>
 <test_depend>ament_lint_common</test_depend>

 <export>
   <build_type>ament_cmake</build_type>
 </export>
</package>
```
:::

:::{tab} Python
```xml
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <test_depend>ament_copyright</test_depend>
 <test_depend>ament_flake8</test_depend>
 <test_depend>ament_pep257</test_depend>
 <test_depend>python3-pytest</test_depend>

 <export>
   <build_type>ament_python</build_type>
 </export>
</package>
```
:::


如果没有自动为您填充，请在`maintainer`行中输入您的姓名和电子邮件。然后，编辑`description`行来总结包：

```xml
<description>Beginner client libraries tutorials practice package</description>
```

然后更新 `license` 行。您可以在[这里](https://opensource.org/licenses/alphabetical)阅读更多关于开放源码许可的信息。由于这个包仅用于实践，所以使用任何许可证都是安全的。我们使用 `Apache License 2.0`：

```xml
<license>Apache License 2.0</license>
```

编辑完成后不要忘记保存。

在许可标记下面，您将看到一些标记名称以 `_depend` 结尾。在这里，您的 `package.xml` 将列出它对其他包的依赖关系，以便 colcon 进行搜索。`my_package` 很简单，并且没有任何依赖项，但是您将在接下来的教程中看到这个空间被利用。

:::{tab} CMake
你们现在都做完了！
:::

:::{tab} Python
`setup.py` 文件包含与 `package.xml` 相同的描述、维护程序和许可字段，因此也需要设置这些字段。它们需要在两个文件中完全匹配。版本和名称（`package_name`）也需要精确匹配，并且应该在两个文件中自动填充。

用您喜欢的文本编辑器打开 `setup.py`。

```python
from setuptools import setup

package_name = 'my_py_pkg'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'my_node = my_py_pkg.my_node:main'
     ],
   },
)
```

编辑 `maintainer`、`maintainer_email` 和 `description` 行以匹配 `package.xml`。

别忘了保存文件。
:::

## 小结

您已经创建了一个包来组织您的代码并使其易于为他人使用。

您的软件包将自动填充必要的文件，然后使用 `colcon` 进行构建，以便可以在本地环境中使用其可执行文件。