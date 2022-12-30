#ifndef POSE_ESTIMATOR_H_
#define POSE_ESTIMATOR_H_
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

class PoseEstimator {
public:
    PoseEstimator() {}

    PoseEstimator(cv::Size img_size);

    // 根据脸部网格顶点，计算头部姿势
    void solve_pose(vector<cv::Point3f> & mesh_points);
    
    // 绘制头部姿势示意图
    void draw_annotation_box(cv::Mat & image); 

    void draw_annotation_box(cv::Mat & image, cv::Mat &rotation_vector, cv::Mat &translation_vector); 
    
    cv::Mat get_rotation_vec();

    // 获取头部角度信息，需要先调用solve_pose
    // 返回的三个角度，分别是pitch(上下）, roll(左右转动）, yaw(歪头)   
    cv::Vec3f get_angle();

    cv::Mat get_translation_vec();

    float get_face_height();

private:
    void solve_pose(cv::Mat & image_points);

    vector<int> indices;

    cv::Size size_;
    
    cv::Mat model_points_;

    float focal_length_;

    cv::Size camera_center_;

    cv::Mat camera_matrix_;

    cv::Mat dist_coeefs_;

    cv::Mat r_vec_;

    cv::Mat t_vec_;
};
#endif 
