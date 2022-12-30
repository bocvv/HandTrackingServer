/*************************************************************************
	> File Name: main.cpp
	> Author: 
	> Mail: 
	> Created Time: 一  8/31 16:57:33 2020
 ************************************************************************/

#include <iostream>
#include "aladin_process.h"

using namespace std;

int main() {
    string bad_pose_path = "./sit_pose_alert.mp3";
    string use_phone_path = "./phone_alert.mp3";
    AladinProcessor *aladin_processor = new AladinProcessor();
    aladin_processor->init(bad_pose_path, use_phone_path);
    aladin_processor->process();

    delete aladin_processor;
}
