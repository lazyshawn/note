## ✨ What should you write in a CMakeLists.txt
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
  "srv/xxx.msg"
)
# 根据 msg/srv 文件生成对应的头文件
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
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

## ✨ What should you write in a package.xml
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

