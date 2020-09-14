## Installing and Configuring Your ROS Environment
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


## Navigating the ROS Filesystem
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


## Creating a ROS Package
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
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CATKIN_IGNORE      -- Optional empty file to exclude package_n from being processed
      CMakeLists.txt
      package.xml
    ...
  build/                 -- BUILD SPACE
    CATKIN_IGNORE        -- Keeps catkin from walking this directory
  devel/                 -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin             
    env.bash
    setup.bash
    setup.sh
    ...
```

### Creating a catkin Package
`catkin_create_pkg` require that you give a package\_name and
optionally a list of a dependencies on which that package depends:
```bash
catkin_create_pkg <package_name> [depend1] [depend2]
```


## Problems & Solutions
1. When run `sudo rosdep init`, it occur `sudo: rosdep: command not found`.
```bash
sudo pip3 install -U rosdep
```

## References
1. [ROS/Tutorials -- ROS Wiki](http://wiki.ros.org/ROS/Tutorials)

