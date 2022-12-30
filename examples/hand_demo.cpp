/*************************************************************************
	> File Name: test.cpp
	> Author: 
	> Mail: 
	> Created Time: 四  8/13 13:23:46 2020
 ************************************************************************/

#include<iostream>
#include "model_manager.h"
#include "ofxUDPManager.h"
#include <opencv.hpp>
#include <core/base.hpp>

#define PORT 8123

using namespace std;

int main() {
    HandManager hand_mgr;
    hand_mgr.setup(PORT);
    bool grab_frames = true;
    while (grab_frames) {
        // drawing landmarks at the white image
        hand_mgr.white_image = cv::Mat(cv::Size(hand_mgr.img_width, \
            hand_mgr.img_height), CV_8UC3, cv::Scalar(206, 206, 205));
        
        // get the udp protobuf message and process
        hand_mgr.update();
        hand_mgr.draw();
        hand_mgr.infer_gesture();

        // draw the rectangle of hand.
        cv::cvtColor(hand_mgr.white_image, hand_mgr.white_image, cv::COLOR_BGR2RGB);
        cv::imshow("hand_track", hand_mgr.white_image);

        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255)
            grab_frames = false;
    }

    return 0;
}
