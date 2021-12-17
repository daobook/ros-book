# 安装 ROS2

详细安装方法见 [ros2-docs](https://daobook.github.io/ros2-docs/xin)。

````{hint}
如果 `raw.githubusercontent.com` 报错，可以尝试：

```sh
sudo curl -sSL https://raw.staticdn.net/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

然后将存储库添加到源代码列表中。

```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
````

````{hint}
在国内 `wget` 网站的 `https://raw.githubusercontent.com` 内容很慢，可以改用：`https://raw.staticdn.net`。

比如 `wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos` 改用为：

```sh
wget https://raw.staticdn.net/ros2/ros2/galactic/ros2.repos
```
````
