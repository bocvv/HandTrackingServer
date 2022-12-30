/*************************************************************************
	> File Name: test.cpp
	> Author: 
	> Mail: 
	> Created Time: 四  8/13 13:23:46 2020
 ************************************************************************/

#include <iostream>
#include "model_manager.h"
#include "ofxUDPManager.h"
#include <opencv.hpp>
#include <core/base.hpp>
#include <thread>
#include <unistd.h>

#define PORT 8767

using namespace std;

cv::Mat g_frame;

int read_from_mjpg_streamer() {
    cv::VideoCapture cap;
    cap.open("http://10.211.55.4:8123/?action=stream?dummy=param.mjpg");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 25);

    while (1) {
        cap >> g_frame;
        // cv::flip(g_frame, g_frame, /*flipcode=HORIZONTAL*/ 1);
    }
}

int main() {
    // open mjpg stream
    std::thread t_read(read_from_mjpg_streamer);
    t_read.detach();
    sleep(2);

    UpperBodyManager upper_body_mgr;
    upper_body_mgr.setup(PORT);
    bool grab_frames = true;
    while (grab_frames) {
        // drawing landmarks at the white image
        // upper_body_mgr.white_image = cv::Mat(cv::Size(upper_body_mgr.img_width, \
            upper_body_mgr.img_height), CV_8UC3, cv::Scalar(206, 206, 205));

        if (g_frame.empty()) break;  // End of video.
        cv::Mat camera_frame(640, 480, g_frame.depth());
        cv::transpose(g_frame, camera_frame);
        cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ -1);
        upper_body_mgr.white_image = camera_frame;
        
        // get the udp protobuf message and process
        upper_body_mgr.update();
        upper_body_mgr.draw();

        // draw the rectangle of hand.
        cv::imshow("upper_body_pose", upper_body_mgr.white_image);

        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255)
            grab_frames = false;
    }

    return 0;
}
