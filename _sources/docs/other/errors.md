# ROS2 常见错误

## 国内 rosdep 更新失败

解决方法：[本文之后，世上再无rosdep更新失败问题！如果有....小鱼就... (qq.com)](https://mp.weixin.qq.com/s/VGs8oWdhHH6XsHcx21lN4Q)

主要是修改 `/etc/ros/rosdep/sources.list.d/20-default.list` 为：

```sh
# os-specific listings first
yaml https://gitee.com/ohhuo/rosdistro/raw/master/rosdep/osx-homebrew.yaml osx

# generic
yaml https://gitee.com/ohhuo/rosdistro/raw/master/rosdep/base.yaml
yaml https://gitee.com/ohhuo/rosdistro/raw/master/rosdep/python.yaml
yaml https://gitee.com/ohhuo/rosdistro/raw/master/rosdep/ruby.yaml
gbpdistro https://gitee.com/ohhuo/rosdistro/raw/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
```

## ROS 在编译生成自定义消息时报错 ModuleNotFoundError: No module named 'em'

```sh
pip install empy -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## ModuleNotFoundError: No module named 'lark'

```sh
pip install lark -i https://pypi.tuna.tsinghua.edu.cn/simple
```