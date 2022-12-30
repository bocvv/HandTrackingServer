# 背景
这是一个基于mediapipe的手部关键点追踪的线上服务版本，可以部署于服务器。需要结合mediapipe一起使用。  
由于google的mediapipe使用自家bazel工具编译，且难以打包成动态库/静态库供我们调用来集成到自己的开发任务中，这方面google也没有计划提供接口和cmake支持。因此可以将mediapipe封装成一个服务，运行期间向固定端口广播mediapipe的检测结果。客户端应用可以监听该端口的udp包来获取mediapipe的结果数据，从而可以进行下一步应用层的开发。

# 简介
该项目基于mediapipe(https://github.com/google/mediapipe) 实现，将hand_tracking示例封装成一个udp广播服务，可以向固定端口使用udp协议广播包装为protobuf格式的检测结果包。使用socket来完成服务和客户应用之间的通信。

# 环境要求
- macOS/Liunx
- opencv3.x or later
- protobuf 3.12.4
- 其他第三方库在3rdpart/中查看

其中protobuf对版本的要求比较高，原则上不能使用低于3.12.4的protobuf。如果要使用较低版本的protobuf，可以执行以下操作重新编译`mediapipe/framework/formats/wrapper_hand_tracking.proto`:  
```
$ cd mediapipe/framework/formats
$ 注释掉`wrapper_hand_tracking.proto`的import部分避免编译proto时寻找依赖。然后取消主体代码的注释，接着执行下面语句：
$ protoc --cpp_out=.  wrapper_hand_tracking.proto
$ 然后将重新编译生成的`wrapper_hand_tracking.pb.h` 和 `wrapper_hand_tracking.pb.cc` 替换该库中src和include文件下下边的同名文件即可。
```


# 快速开始
## demo

### 修改mediapipe，添加udp服务功能
首先将本库中mediapipe文件夹下的文件移动到mediapipe目录下的对应目录覆盖之前的文件：
```
$ cp -R mediapipe /path/to/mediapipe
```

### 启动mjpg streamer服务
```
$ mjpg_streamer -i 'input_uvc.so -d /dev/video0' -o 'output_http.so -p 8123'
```

### 编译&启动mediapipe的handtracking服务

```
$ cd /path/to/mediapipe
$ bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/hand_tracking:hand_tracking_cpu
$ GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/hand_tracking/hand_tracking_cpu \
  --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt
```


### 启动客户应用demo
```
$ cv /path/to/udp_mediapipe
$ mkdir build && cd build
$ cmake ..
$ make -j4
$ ./hand_demo
```
