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

#define PORT 8768

using namespace std;

int main() {
    ObjectTrackingManager obj_mgr;
    obj_mgr.setup(PORT);
    bool grab_frames = true;
    while (grab_frames) {
        // drawing landmarks at the white image
        obj_mgr.white_image = cv::Mat(cv::Size(obj_mgr.img_width, \
           obj_mgr.img_height), CV_8UC3, cv::Scalar(206, 206, 205));
        
        // get the udp protobuf message and process
        // double t = (double)cv::getTickCount();
        cout << "bo1" << endl;
        obj_mgr.update();
        cout << "bo2" << endl;
        // obj_mgr.draw();
        // t =  ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << t << endl;

        // draw the rectangle of obj.
        // cv::cvtColor(obj_mgr.white_image,obj_mgr.white_image, cv::COLOR_BGR2RGB);
        // cv::imshow("obj_tarcking",obj_mgr.white_image);

        // Press any key to exit.
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255)
            grab_frames = false;
    }

    return 0;
}
