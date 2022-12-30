#include "pose_estimator.h"
#include <fstream>

using namespace cv;

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    
    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));
    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return Vec3f(x, y, z);
    
}

PoseEstimator::PoseEstimator(cv::Size img_size) {
    size_ = img_size;

    model_points_ = (cv::Mat_<float>(68, 3));
    ifstream infile("assets/model.txt");
    string line;
    int i = 0;
    while(getline(infile, line)) {
        int row_id = i % 68;
        int col_id = i / 68;
        if (col_id < 2) {
            model_points_.at<float>(row_id, col_id) = stof(line);
        } else {
            model_points_.at<float>(row_id, col_id) = -stof(line);
        }
        i += 1;
    }
    infile.close();     

    ifstream indices_file("indices.txt");
    while(getline(indices_file, line)) {
        indices.push_back(stoi(line));
    }
    indices_file.close();

    // camera internals
    focal_length_ = size_.width;
    camera_center_ = cv::Size(size_.width / 2, size_.height / 2);
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            focal_length_, 0, camera_center_.width,
            0, focal_length_, camera_center_.height,
            0, 0, 1); 
    // Assuming no lens distortion
    dist_coeefs_ = cv::Mat::zeros(4, 1, CV_32F);
    // Rotation vector and translation vector 
    r_vec_ = (cv::Mat_<double>(3, 1) << 0.01891013, 0.08560084, -3.14392813); 
    t_vec_ = (cv::Mat_<double>(3, 1) << -14.97821226, -10.62040383, -2053.03596872);
}

void PoseEstimator::solve_pose(vector<cv::Point3f> & mesh_points) {
    //cout << indices.size() << endl;
    cv::Mat image_points(indices.size(), 2, CV_32F);
    for (int i = 0; i < indices.size(); i++) {
        image_points.at<float>(i, 0) = mesh_points[indices[i]].x * size_.width;
        image_points.at<float>(i, 1) = mesh_points[indices[i]].y * size_.height;
    }
    
    solve_pose(image_points);
}

void PoseEstimator::solve_pose(cv::Mat & image_points) {
    cv::solvePnP(model_points_, image_points, camera_matrix_, dist_coeefs_, r_vec_, t_vec_, true);
}

cv::Mat PoseEstimator::get_rotation_vec() {
    return r_vec_;
}

Vec3f PoseEstimator::get_angle() {
    Mat rotation_matrix = Mat::zeros(3, 3, CV_32F); 
    cv::Rodrigues(r_vec_, rotation_matrix);
    return rotationMatrixToEulerAngles(rotation_matrix);
}

cv::Mat PoseEstimator::get_translation_vec() {
    return t_vec_;
}

void PoseEstimator::draw_annotation_box(cv::Mat & image) {
    draw_annotation_box(image, r_vec_, t_vec_);
}

float PoseEstimator::get_face_height() {
    int rear_size = 75;
    int rear_depth = 0;
    cv::Mat point_3d = (cv::Mat_<float>(10, 3) <<
            -rear_size, -rear_size, rear_depth,
            -rear_size, rear_size, rear_depth);
    vector<cv::Point2f> point_2d;
    cv::projectPoints(point_3d, r_vec_, t_vec_, camera_matrix_, dist_coeefs_, point_2d);
    return sqrt((point_2d[0].x - point_2d[1].x) * (point_2d[0].x - point_2d[1].x) + 
        (point_2d[0].y - point_2d[1].y) * (point_2d[0].y - point_2d[1].y));  
}

void PoseEstimator::draw_annotation_box(cv::Mat & image, cv::Mat &rotation_vector, cv::Mat &translation_vector) {
    int rear_size = 75;
    int rear_depth = 0;
    int front_size = 100;
    int front_depth = 100;
    cv::Mat point_3d = (cv::Mat_<float>(10, 3) <<
            -rear_size, -rear_size, rear_depth,
            -rear_size, rear_size, rear_depth,
            rear_size, rear_size, rear_depth,
            rear_size, -rear_size, rear_depth,
            -rear_size, -rear_size, rear_depth,
            -front_size, -front_size, front_depth,
            -front_size, front_size, front_depth,
            front_size, front_size, front_depth,
            front_size, -front_size, front_depth,
            -front_size, -front_size, front_depth);
    vector<cv::Point2f> point_2d;
    cv::projectPoints(point_3d, rotation_vector, translation_vector, camera_matrix_, dist_coeefs_, point_2d);
    //vector<vector<cv::Point2f>> point_2d_v;
    cv::Mat point_mat(point_2d.size(), 2, CV_32S);
    for (int i = 0; i < point_2d.size(); i++) {
        point_mat.at<int>(i, 0) = (int)point_2d[i].x;
        point_mat.at<int>(i, 1) = (int)point_2d[i].y;
    }
    vector<cv::Mat> point_2d_v;
    point_2d_v.push_back(point_mat); 
    cv::polylines(image, point_2d_v, true, cv::Scalar(255, 0, 0), 2, CV_AA); 
    cv::line(image, point_2d[1], point_2d[6], cv::Scalar(255, 0, 0), 2, CV_AA);
    cv::line(image, point_2d[2], point_2d[7], cv::Scalar(255, 0, 0), 2, CV_AA);
    cv::line(image, point_2d[3], point_2d[8], cv::Scalar(255, 0, 0), 2, CV_AA);
}


