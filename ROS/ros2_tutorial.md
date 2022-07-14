<!-- TOC GFM -->

* [✨ How to Write a ROS2 Node](#-how-to-write-a-ros2-node)
* [✨ How to Create Your ROS2 Pkg](#-how-to-create-your-ros2-pkg)
  - [What should you write in a CMakeLists.txt](#what-should-you-write-in-a-cmakeliststxt)
  - [What should you write in a package.xml](#what-should-you-write-in-a-packagexml)
  - [How to Link Library in a ROS2 pkg](#how-to-link-library-in-a-ros2-pkg)
* [✨ Useful Command-line Interface](#-useful-command-line-interface)

<!-- /TOC -->

✨ Basic ROS2 Concepts
----------------------
#### action
`action` 的作用类似于 `service`，不同点在于 `action` 能提供反馈，而且是可抢占的。
```bash
# goal
---
# result
---
# feedback
```


## ✨ How to Write a ROS2 Node

## ✨ How to Create Your ROS2 Pkg
`ros2 pkg create <pkg_name> [--param value]`:
* `build-type`: `ament_cmake` for cpp
* `dependencies`: 依赖包
* `node-name`: 默认创建的节点

### What should you write in a CMakeLists.txt
> 需要的 ROS 包
```CMake
find_package(rclcpp REQUIRED)
```

> 话题topic / 服务service / 动作action 中使用的消息文件
```CMake
set(msg_files
  "msg/xxx.msg"
)
set(srv_files
  "srv/xxx.srv"
)
set(act_files
  "action/xxx.action"
)
# 根据 msg/srv 文件生成对应的头文件
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${act_files}
)
# 编译运行时的 interface 依赖 (使用当前包的消息时需要)
ament_export_dependencies(rosidl_default_runtime)
```

> 将节点编译成可执行文件
```CMake
# 生成可执行文件
add_executable(demo src/listen.cpp src/foo.cpp)
# 添加头文件位置
target_include_directories(demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 为节点链接需要的包
ament_target_dependencies(demo rclcpp std_msgs)
# 使用 interface
rosidl_target_interfaces(demo ${PROJECT_NAME} rosidl_typesupport_cpp)
```

> 将节点编译成链接库
```CMake
# ros2 run 通过该方法寻找可执行文件
install(TARGETS
  demo
  demo_
  DESTINATION lib/${PROJECT_NAME})
```

### What should you write in a package.xml
> 需要的 ROS 包
```xml
<!-- ROS2 的编译工具 -->
<buildtool_depend>ament_cmake</buildtool_depend>
<!-- 需要的 ROS 包 -->
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<!-- 生成 interface 的工具 -->
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### How to Link Library in a ROS2 pkg
> 更适合 ROS 的解决方案是使用 [plugins](
https://docs.ros.org/en/foxy/Tutorials/Pluginlib.html).

ROS2 是以 [colcon](https://colcon.readthedocs.io/en/released/index.html)
为包构建工具的，在 ROS2 中链接库文件的方式也是基于包进行链接的，即在 ROS2 的包
<1> 中导出了链接库文件和依赖后，其他包只需要将包 <1> 作为依赖就能使用其链接库文
件。

例如，下面我们在`createlib`中将`hello.cpp`创建为库，并在`uselib`中使用它。
```bash
createlib             |  uselib                 
├── CMakeLists.txt    |  ├── CMakeLists.txt
├── include           |  ├── include       
│   └── createlib     |  │   └── uselib    
│       └── hello.h   |  ├── package.xml   
├── package.xml       |  └── src           
└── src               |      └── main.cpp  
    ├── hello.cpp     |  
    └── main.cpp      |  
```

1. 编译`createlib`时的`CMakeLists.txt`文件如下。导出后的库文件和对应的头文件可以
   在路径`ros2ws/install/createlib`下找到。
```CMake
add_library(hello_lib SHARED
  src/hello.cpp
)
target_include_directories(hello_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 为了使下游文件可以访问
ament_export_targets(hello_lib HAS_LIBRARY_TARGET)
# 注册 导出头文件
install(
  DIRECTORY include/  # 需要共享的头文件位置
  DESTINATION include
)
# 注册 导出库文件
install(
  TARGETS hello_lib   # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT hello_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# createlib 包后续的 target 链接 hello_lib
# target_link_libraries(bar hello_lib)
```

2. 在`uselib`中需要在`CMakeLists.txt`和`package.xml`中将`createlib`添加为依赖。
```CMake
# CMakeLists.txt
find_package(createlib REQUIRED)

add_executable(libbin
  src/main.cpp
)
# 添加头文件位置
target_include_directories(libbin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 为节点链接需要的包
ament_target_dependencies(libbin rclcpp createlib)

install(TARGETS
  libbin
  DESTINATION lib/${PROJECT_NAME})

# package.xml
<depend>createlib</depend>
```


✨ Useful Command Line Interface
--------------------------------
1. `ros2 interface show <interface>`
1. `ros2 <command> list [-t]`

