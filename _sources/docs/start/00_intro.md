# ROS 简介

参考：[Documentation - ROS Wiki](http://wiki.ros.org/)

ROS（Robot Operating System）是一种用于编写机器人软件的灵活框架。它是一个工具（tools）、库（libraries）和约定（conventions）的集合，旨在简化在各种机器人平台上创建复杂而健壮的机器人行为的任务。

ROS
:   ROS 是适用于机器人的开源的元操作系统。它提供了操作系统应有的服务，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理。它也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。在某些方面 ROS 相当于一种“机器人框架（robot frameworks）”类似的“机器人框架”有：[Player](http://playerstage.sf.net/)，[YARP](http://eris.liralab.it/yarp/)，[Orocos](http://www.orocos.org/)，[CARMEN](http://carmen.sourceforge.net/)，[Orca](http://orca-robotics.sourceforge.net/)，[MOOS](http://www.robots.ox.ac.uk/~pnewman/TheMOOS/index.html)和 [Microsoft Robotics Studio](http://msdn.microsoft.com/en-us/robotics/default.aspx)。

    ROS 运行时的“graph”是一种基于 ROS 通信基础结构的松耦合点对点进程网络。ROS 实现了几种不同的通信方式，包括基于同步 RPC 样式通信的[服务（services）](http://wiki.ros.org/Services)机制，基于异步流媒体数据的[话题（topics）](http://wiki.ros.org/Topics)机制以及用于数据存储的[参数服务器（Parameter Server）](http://wiki.ros.org/Parameter%20Server)。想更加深入的解释这些概念，请参见[概念综述](http://wiki.ros.org/ROS/Concepts)。

    ROS 并不是一个实时的框架，但 ROS 可以嵌入实时程序。Willow Garage 的 PR2 机器人使用了一种叫做[pr2_etherCAT](http://wiki.ros.org/pr2_etherCAT)的系统来实时发送或接收ROS消息。ROS也可以[与Orocos实时工具包无缝集成](http://www.willowgarage.com/blog/2009/06/10/orocos-rtt-and-ros-integrated)。


    ````{panels}
    :container: +full-width text-center
    :header: w3-pale-blue w3-wide w3-card-4 
    :column: col-lg-12 px-2 py-2
    :body: text-justify w3-padding
    :card: w3-pale-red w3-card
    ---
    ROS 的目标
    ^^^
    ROS 的主要目标是为机器人研究和开发提供代码**复用**的支持。ROS 是一个分布式的进程（也就是节点）框架，这些进程被封装在易于被分享和发布的程序包和功能包集中。ROS 也支持一种类似于代码储存库的联合系统，这个系统也可以实现工程的协作及发布。这个设计可以使一个工程的开发和实现从文件系统到用户接口完全独立决策（不受 ROS 限制）。同时，所有的工程都可以被 ROS 的基础工具整合在一起。

    为了支持分享和协作的主要目的，ROS 框架也有其它几个目标：

    - 小型化：ROS 尽可能设计的很小 -- 我们不封装您的 `main()` 函数 -- 所以为 ROS 编写的代码可以轻松的在其它机器人软件平台上使用。由此得出的必然结论是 ROS 可以轻松集成在其它机器人软件平台：ROS 已经可以与 OpenRAVE，Orocos 和 Player（非常适合简洁的非铰接的移动平台）集成。
    - ROS 不敏感库：ROS 的首选开发模型都是用不依赖 ROS 的干净的库函数编写而成。
    - 语言独立：ROS 框架可以简单地使用任何的现代编程语言实现。我们已经实现了 [Python版本](http://wiki.ros.org/rospy)，[C++版本](http://wiki.ros.org/roscpp)和 [Lisp版本](http://wiki.ros.org/roslisp)。同时，我们也拥有 Java 和 Lua 版本的实验库。
    - 方便测试：ROS 内建一个了叫做 `rostest` 的单元/集成测试框架，可以轻松安装或卸载测试模块。
    - 可扩展：ROS 可以适用于大型运行时系统和大型开发进程。
    ````

## ROS2

参考：[ROS 2 Documentation — ROS 2 Documentation: Galactic documentation](https://docs.ros.org/en/galactic/index.html)

ROS 是一套用于构建机器人应用程序的软件库和工具。从驱动程序到最先进的算法，以及强大的开发工具，ROS 拥有您下一个机器人项目所需的一切。而且都是开源的。

自 ROS 于 2007 年启动以来，机器人和 ROS 社区发生了很多变化。ROS 2 项目的目标是适应这些变化，利用 ROS 1 的优点并改进缺点。

如果你正在寻找关于 ROS 1 的文档，请查看 [ROS wiki](https://wiki.ros.org/)。

导航：

- [](ros:install) 页面将帮助您首次安装 ROS 2。您可以选择适合您需要的平台以及安装类型和分发版。
- [](ros:docs-guide) 解释了 ROS 1 和 ROS 2 文档基础结构。它有助于理解具体资源在哪里，如何提问，以及维护哪些站点。
- [](ros:tutorials) 将引导你完成小型项目和 ROS2 的示例使用，所以你可以通过实际使用工具来学习诀窍。它们是根据必要技能的发展来组织的，这使它成为新用户开始的最佳场所。
- [](ros:how-to-guides) 回答了你所有的“我如何……?”问题尽可能简短而直接，这样你就可以在不需要通过教程的叙述性质的背景下找到你需要的信息。
- [](ros:concepts) 是关于核心 ROS 2 概念的高级解释和背景信息，应该为教程中涉及的主题提供一些上下文。
- [](ros:contributing) 是使 ROS 2 伟大的原因！我们有关于贡献新的 ROS 2 内容以及将现有内容从 ROS 1 迁移到 ROS 2 的最佳实践和方法指南。
- [寻求帮助](ros:contact) 总是被鼓励的。在这里，你会找到几种方法来回答你的问题或开始一场讨论。

## 关于 ROS2

* [Releases](https://docs.ros.org/en/galactic/Releases.html#releases)：过去，现在和未来的 ROS 2 发行版，包括发行版之间的日期和变化。
* [Features](https://docs.ros.org/en/galactic/Features.html#features)：当前 ROS 2 版本的一部分功能，以及详细介绍每个新功能的内容。
* [Roadmap](https://docs.ros.org/en/galactic/Roadmap.html#roadmap)：一组尚未实现但即将实现的 ROS 2 的特性想法。
* [ROSCon talks](https://docs.ros.org/en/galactic/ROSCon-Content.html#roscon)：关于整个社区对 ROS 2 的最新使用和更新的介绍。
* [Project governance](https://docs.ros.org/en/galactic/Governance.html#governance)：由技术指导委员会处理的，您可以在这里了解更多。
* [Marketing](https://docs.ros.org/en/galactic/Marketing.html#marketing)：推广 ROS 2 的资料可从此网页下载。
* [ROS 2 Design](http://design.ros2.org/)：包含了许多关于 ROS2 背后的设计决策的文章。

更多内容见：[ros2/ros2_documentation: ROS 2 docs repository (github.com)](https://github.com/ros2/ros2_documentation)。