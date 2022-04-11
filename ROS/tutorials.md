## ✨ Summary
### What should you write in a CMakeLists.txt for a ros-package
> 需要的 ROS 包
```CMake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

> 话题topic / 服务service / 动作action 中使用的消息文件
```CMake
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
)
```

> 将节点编译成可执行文件的方法
```CMake
add_executable(listen src/listen.cpp src/foo.cpp)
target_link_libraries(listen ${catkin_LIBRARIES})
add_dependencies(listen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

## ✨ Installing and Configuring Your ROS Environment
### Install ROS
Before starting these tutorials please complete installation as described in the
[ROS installation instructions](http://wiki.ros.org/ROS/Installation).

### Managing Your Environment
You must source this script in every terminal you use ROS in.
For convenience, you can add this to your .rc file.
```bash
# For bash
source /opt/ros/noetic/setup.bash
# For zsh
source /opt/ros/noetic/setup.zsh
```

This is required because ROS relies on the notion of combining spaces
using the shell environment. 
This makes developing against different versions of ROS
or against different sets of packages easier.

If you are ever having problems finding or using your ROS packages
make sure that you have your environment properly setup.
A good way to check is to ensure that environment variables
like ROS\_ROOT and ROS\_PACKAGE\_PATH are set:
```shell
$ printenv | grep ROS
```

After create ROS workspace using `catkin_make`, 
you can source a setpu.\*sh inside the `devel` folder to
overlay this workspace on the top of your environment,
i.e, add the directory you're in to ROS\_PACKAGE\_PATH.


## ✨ Navigating the ROS Filesystem
> **Description:** Introduces ROS filesystem concepts,
and covers using the `roscd`, `rosls`, and `rospack` commandline tools.

### Usage
```bash
$ rospack find [pkg_name]
$ roscd [pkg_name[/subdir]]
$ rosls [pkg_name[/subdir]]
```
1. `roscd log` command will take you to the folder where ROS stores log files.
1. Support Tab completion.


## ✨ Creating a ROS Package
> **Description:** Using roscreate-pkg or catkin to create a new package,
and rospack to list package dependencies.

### What makes up a catkin Package
For a package to be considered a catkin package it must meet a few requirements:
* The package must contain a catkin compliant `package.xml` file.
* The package must contain a `CMakeLists.txt` which uses catkin.
* Each package must have its own folder.

### Packages in a catkin Workspace
A trivial workspace might look like this: 
```bash
workspace_folder/      -- WORKSPACE
  src/                 -- SOURCE SPACE
  | CMakeLists.txt     -- 'Toplevel' CMake file, provided by catkin
  | package_1/
  |   CATKIN_IGNORE    -- Optional empty file to exclude package_1 from being processed
  |   CMakeLists.txt   -- CMakeLists.txt file for package_1
  |   package.xml      -- Package manifest for package_1
  |   src/
  |   msg/
  |   srv/
  |   launch/
  build/               -- BUILD SPACE
  devel/               -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
  install/             -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
```

### Creating a catkin Package
`catkin_create_pkg` require that you give a package\_name and
optionally a list of a dependencies on which that package depends:
```bash
catkin_create_pkg <package_name> [depend1] [depend2]
```
Here's some common dependencies:
`std_msgs`, `rospy`, `roscpp`.

#### package.xml
The generated [package.xml](https://wiki.ros.org/catkin/package.xml) should be
in your new package. It describe the basic information of the package, like
name, version, description, maintainer, license, dependencies, and etc.

The dependencies are split into build_depend, buildtool_depend, exec_depend, 
test_depend.
* **Build Tool Dependences** `<buildtool_depend>`: specify build system tools 
which this package needs to build itself. Typically the only build tool needed
is catkin.
* **Build Dependences** `<build_depend>`: specify which packages are needed to
build this package.This can be including headers from these packages at
compilation time, linking against libraries from these packages or requiring
any other resource at build time. (`find_package()`)
* **Run Dependences** `<exec_depend>`: specify which packages are needed to run
code in this package, or build libraries against this package. This is the case
when you depend on shared libraries or transitively include their headers in
public headers in this package. (`catkin_package([CATKIN_]DEPENDS)`)
* **Test Dependences** `<test_depend>`:  specify only additional dependencies
for unit tests. They should never duplicate any dependencies already mentioned
as build or run dependencies.


## ✨ Basic ROS Concepts
* [Nodes](https://wiki.ros.org/Nodes): A node is an executable that uses ROS to
communicate with other nodes.
* [Messages](https://wiki.ros.org/Messages): ROS data type used when subscribing
or publishing to a topic.
* [Topics](https://wiki.ros.org/Topics): Nodes can publish messages to a topic
as well as subscribe to a topic to receive messages.
* [Master](https://wiki.ros.org/Master): Name service for ROS (i.e. helps nodes
find each other).


## ✨ Run ROS tasks
### rosrun
rosrun allows you to use the package name to directly run a node within a
package.
```bash
rosrun [package_name] [node_name] <optional_param>
```

### roslaunch
[roslaunch](http://wiki.ros.org/roslaunch) starts nodes as defined in a launch
file.
```bash
roslaunch [package] [filename.launch]
```
* 不以"/"开头，表示使用的是相对命名空间。


## ✨ Creating a msg/srv
### Common step for msg and srv
**Step 1**: define a new msg/srv file in the `msg`/`srv` directory of the package.

**Step 2**: open package.xml, and make sure these two lines are in it and uncommented:
```bash
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

**Step 3**: open CMakeLists.txt. Add the `message_generation` dependency to the
find_package call so that you can generate messages. You can do this by simply
adding message_generation to the list of COMPONENTS such that it looks like this:
```bash
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

**Step 4**: make sure you export the `message_runtime` dependency.
```bash
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
)
```

**Step 5**: add message/service files. By adding the .msg/.srv files manually,
we make sure that CMake knows when it has to reconfigure the project after you
add other .msg/.srv files.
```bash
add_message_files(
  FILES
  Message.msg
)
add_service_files(
  FILES
  Service.srv
)
```

**Step 6**:  ensure the `generate_messages()` function is called. Add any
packages you depend on which contain .msg files that your messages use
(`定义了你使用的.msg文件中使用的消息类型的包` in this case std_msgs).
```bash
generate_messages(
  DEPENDENCIES
  std_msgs    # Or other packages containing msgs
)
```

### Writing a simple Publisher and Subscriber
```cpp
/**********************************
* std_msgs/String.msg
* String data
**********************************/
#include "std_msgs/String.h"   // generated from the .msg file in the package
// Publisher
ros::Publisher chatter_pub = NodeHandle.advertise<std_msgs::String>("chatter", 1000);
msg.data = "hello world";
chatter_pub.publish(msg);
// Subscriber
void chatterCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
ros::spin();    // receive msg and handle callback function
```

### Writing a simple Service and Client
```cpp
/**********************************
* beginner_tutorials/AddTwoInts.srv
* int a
* int b
* ---
* int sum
**********************************/
#include "beginner_tutorials/AddTwoInts.h"
// Service
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
ros::ServiceServer service = NodeHandle.advertiseService("add_two_ints", add);
ros::spin();
// Client
ros::ServiceClient client = NodeHandle.serviceClient<beginner_tutorials::AddTwoInts>(
  "add_two_ints");
beginner_tutorials::AddTwoInts srv;
if (client.call(srv)) {
  ROS_INFO("Sum: %ld", (long int)srv.response.sum);
}
```


### Building your node
Add these few lines to the bottom of your CMakeLists.txt:
> The [pkg]_generate_messages_cpp is a target created by generate_messages.
```bash
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker [pkg]_generate_messages_cpp)
add_dependencies(tlaker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```



## ✨ Problems & Solutions
* When run `sudo rosdep init`, it occur `sudo: rosdep: command not found`.
```bash
sudo pip3 install -U rosdep
```
* If roscore does not initialize and sends a message about lack of permissions,
probably the ~/.ros folder is owned by root, change recursively the ownership of
that folder with:
```bash
sudo chown -R <your_username> ~/.ros
```


## ✨ References
1. [ROS/Tutorials -- ROS Wiki](http://wiki.ros.org/ROS/Tutorials)

