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

#define PORT 8765 

using namespace std;

int main() {
    MultiHandManager multi_hand_mgr;
    multi_hand_mgr.setup(PORT);
    bool grab_frames = true;
    while (grab_frames) {
        // drawing landmarks at the white image
        multi_hand_mgr.white_image = cv::Mat(cv::Size(multi_hand_mgr.img_width, \
           multi_hand_mgr.img_height), CV_8UC3, cv::Scalar(206, 206, 205));
        
        // get the udp protobuf message and process
        // double t = (double)cv::getTickCount();
        multi_hand_mgr.update();
        multi_hand_mgr.draw();
        // t =  ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << t << endl;

        // draw the rectangle ofmulti_hand.
        cv::cvtColor(multi_hand_mgr.white_image,multi_hand_mgr.white_image, cv::COLOR_BGR2RGB);
        cv::imshow("multi_hand_tracking",multi_hand_mgr.white_image);

        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255)
            grab_frames = false;
    }

    return 0;
}
