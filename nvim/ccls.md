## Standard C++ header not found
This may be caused by a wrong `.ccls` file.
Use following instead to specify your c++ `include` path:
```ccls
-I/usr/include/c++/9
-I/usr/include/x86_64-linux-gnu/c++/9
```
