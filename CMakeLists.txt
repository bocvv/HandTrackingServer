cmake_minimum_required(VERSION 2.8.10)
project(ofxNetworks)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")

include_directories(include)
include_directories(include/udp)
include_directories(src/strategy)
include_directories(src/smtp)
include_directories(3rdparty/boost/include)
include_directories(3rdparty/utf8/include)
include_directories(3rdparty/protobuf/include)
include_directories(/usr/local/Cellar/opencv@3/3.4.10_4/include/opencv2)
include_directories(/usr/local/Cellar/opencv@3/3.4.10_4/include)

link_directories(3rdparty/boost/lib)
link_directories(3rdparty/protobuf/lib)
link_directories(/Users/bocvv/neteasy/lib)
    
set(ofxNet_srcs
    src/udp/ofUtils.cpp
    src/udp/ofxNetworkUtils.cpp
    src/udp/ofLog.cpp
    src/udp/ofFileUtils.cpp
    src/udp/ofxUDPManager.cpp
)

set(strategy_srcs
    src/strategy/pose_estimator.cpp
    src/strategy/stabilizer.cpp
    src/strategy/scorer.cpp
)

set(demo_srcs
    ${ofxNet_srcs}
    src/model_manager.cpp
    src/wrapper_model.pb.cc
)

option(SAVE_RAW_VIDEO "save the raw video captured by camera or mjpg" OFF)
option(WITH_PHONE_CHECKING "check the distance between hand and cell phone" OFF)
if(SAVE_RAW_VIDEO)
    add_definitions(-DSAVE_RAW_VIDEO)
endif()
if(WITH_PHONE_CHECKING)
    add_definitions(-DWITH_PHONE_CHECKING)
endif()

add_library(ofxNetwork ${ofxNet_srcs})
add_executable(hand_demo examples/hand_demo.cpp ${demo_srcs})
add_executable(face_demo examples/face_demo.cpp ${demo_srcs})
add_executable(upper_body_demo examples/upper_body_demo.cpp ${demo_srcs})
add_executable(iris_tracking_demo examples/iris_tracking_demo.cpp ${demo_srcs})
add_executable(multi_hand_demo examples/multi_hand_demo.cpp ${demo_srcs})
add_executable(object_tracking_demo examples/obj_tracking_demo.cpp ${demo_srcs})
add_executable(aladin src/main.cpp src/aladin_process.cpp ${demo_srcs} ${strategy_srcs} src/smtp/smtp.cpp)

target_link_libraries(ofxNetwork boost_filesystem boost_system protobuf opencv_world.3.4)
target_link_libraries(hand_demo boost_filesystem boost_system protobuf opencv_world.3.4)
target_link_libraries(face_demo boost_filesystem boost_system protobuf opencv_world.3.4)
target_link_libraries(upper_body_demo boost_filesystem boost_system protobuf opencv_world.3.4)
target_link_libraries(iris_tracking_demo boost_filesystem boost_system protobuf opencv_world.3.4)
target_link_libraries(multi_hand_demo boost_filesystem boost_system protobuf opencv_world.3.4)
target_link_libraries(object_tracking_demo boost_filesystem boost_system protobuf opencv_world.3.4)

add_compile_options(-ldl -lm  -lpthread)
target_link_libraries(aladin boost_filesystem boost_system protobuf opencv_world.3.4)
