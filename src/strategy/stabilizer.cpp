#include "stabilizer.h"

using namespace cv;

Stabilizer::Stabilizer() {
}

Stabilizer::Stabilizer(int state_num, int measure_num, float cov_process, float cov_measure) : state_num_(state_num), measure_num_(measure_num) {
    filter_ = new cv::KalmanFilter(state_num, measure_num, 0); 
    state_ = cv::Mat::zeros(state_num, 1, CV_32F); 
    measurement_ = Mat::zeros(measure_num, 1, CV_32F);
    prediction_ = Mat::zeros(state_num, 1, CV_32F);

    if (measure_num == 1) {
        filter_->transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
	std::cout << filter_->transitionMatrix;
        filter_->measurementMatrix = (Mat_<float>(1, 2) << 1, 1);
	std::cout << filter_->measurementMatrix;
	filter_->processNoiseCov = (Mat_<float>(2, 2) << 1, 0, 1, 0);
	filter_->processNoiseCov *= cov_process;
	std::cout << filter_->processNoiseCov;
	filter_->measurementNoiseCov = (Mat_<float>(1, 1) << 1); 
	filter_->measurementNoiseCov *= cov_measure;
	std::cout << filter_->measurementNoiseCov;
        //setIdentity(filter_->processNoiseCov, Scalar::all(cov_process));
        //setIdentity(filter_->measurementNoiseCov, Scalar::all(cov_measure));
        //setIdentity(filter_->errorCovPost, Scalar::all(1));
    }
}

void Stabilizer::update(float measurement) {
    prediction_ = filter_->predict();
     
    measurement_.at<float>(0, 0) = measurement;         

    filter_->correct(measurement_);

    state_ = filter_->statePost;
}

cv::Mat Stabilizer::get_state() {
    return filter_->statePost; 
}
