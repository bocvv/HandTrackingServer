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
    IrisTrackingManager iris_mgr;
    iris_mgr.setup(PORT);
    bool grab_frames = true;
    while (grab_frames) {
        // drawing landmarks at the white image
        iris_mgr.white_image = cv::Mat(cv::Size(iris_mgr.img_width, \
            iris_mgr.img_height), CV_8UC3, cv::Scalar(206, 206, 205));
        
        // get the udp protobuf message and process
        iris_mgr.update();
        iris_mgr.draw();

        // draw the rectangle of hand.
        cv::cvtColor(iris_mgr.white_image, iris_mgr.white_image, cv::COLOR_BGR2RGB);
        cv::imshow("iris_tracking", iris_mgr.white_image);

        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255)
            grab_frames = false;
    }

    return 0;
}
