#ifndef STABILIZER_H_
#define STABILIZER_H_

#include <opencv2/opencv.hpp>

class Stabilizer {
public:
    Stabilizer();

    Stabilizer(int state_num, int measure_num, float cov_process = 0.0001, float cov_measure=0.1); 

    void update(float mesurement); 

    cv::Mat get_state();

private:

    int state_num_; 
     
    int measure_num_; 

    cv::KalmanFilter* filter_;

    cv::Mat state_;

    cv::Mat prediction_;

    cv::Mat measurement_;
};

#endif
