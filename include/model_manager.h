/*************************************************************************
	> File Name: model_manager.h
	> Author: Bowei Wang
	> Mail: wangbw@rd.neteasy.com
	> Created Time: 五  8/28 18:05:15 2020
 ************************************************************************/

#ifndef _MODEL_MANAGER_H
#define _MODEL_MANAGER_H


#include "ofxUDPManager.h"
#include "wrapper_model.pb.h"
#include <vector>
#include <opencv.hpp>
#include <core/base.hpp>


#define HAND_POINT_NUMBER 21 
#define MAX_HAND_NUMBER 4
#define EYE_CONTOUR_POINT_NUM 71
#define IRIS_POINT_NUM 5
#define FACE_POINT_NUMBER 468
#define MAX_FACE_NUMBER 4
#define UPPER_BODY_POINT_NUM 31
#define PHONE_DISTANCE 150  // pixel value


class ModelManager {
public:
	ModelManager() {}
	virtual ~ModelManager();

	virtual void setup(int PORT) = 0;
	virtual void update() = 0;
	virtual void draw() = 0;

	// udp manager
	ofxUDPManager udp_mgr;
    char udp_message[100000];
    int size_n;

	// point and rect used for drawing.
    typedef struct _fVec3f {
        float x, y, z;
    }fVec3f;

    // raw image's shape.
    cv::Mat white_image;
};

class HandManager : public ModelManager {
public:
	HandManager() {}
    ~HandManager() {}

	void setup(int PORT) override;
	void update() override;
	void draw() override;
    void infer_gesture();

    // protobuf data blob.
    ::mediapipe::WrapperHandTracking* wrapper;

	// points vector for hand detection result
    std::vector<cv::Point3f> hand_pts;
    std::vector<cv::Point3f> hand_center_pts;

    typedef struct _HandRect {
        float hand_cx;
        float hand_cy;
        float hand_w;
        float hand_h;
        float rotation = 0; // in radians
    }HandRect;
    HandRect hand_rect;

    int hand_num;

	// raw image's shape.
    int img_width = 640;
    int img_height = 480;

    // hand gesture
    typedef enum _gesture {
        SINGLE_FINGER = 0,
        DOUBLE_FINGER = 1,
        OTHERS        = 2
    }gesture;

    gesture finger_pose;
};

class MultiHandManager : public ModelManager {
public:
    // TODO: init constructor with initialize
    MultiHandManager() {}
    ~MultiHandManager() {}

    void setup(int PORT) override;
    void update() override;
    void draw() override;

    // protobuf data blob.
    ::mediapipe::WrapperMultiHand* wrapper;

    // point and rect used for drawing.
    std::vector<cv::Point3f> hand_pts;
    std::vector<std::vector<cv::Point3f> > hand_pts_group;
    std::vector<cv::Point3f> hand_center_pts;

    typedef struct _HandRect {
        float hand_cx;
        float hand_cy;
        float hand_w;
        float hand_h;
        float rotation = 0; // in radians
    }HandRect;
    HandRect hand_rect;

    int hand_num;

    // raw image's shape.
    int img_width = 640;
    int img_height = 480;
};

class FaceManager : public ModelManager {
public:
    // TODO: init constructor with initialize
    FaceManager() {}
    ~FaceManager() {}

    void setup(int PORT) override;
    void update() override;
    void draw() override;

    // protobuf data blob.
    ::mediapipe::WrapperFaceMesh* wrapper;

    // point and rect used for drawing.
    std::vector<cv::Point3f> face_pts;
    std::vector<std::vector<cv::Point3f> > face_pts_group;

    typedef struct _FaceRect {
        float face_cx;
        float face_cy;
        float face_w;
        float face_h;
        float rotation = 0; // in radians
    }FaceRect;
    FaceRect face_rect;

    int face_num;

    // raw image's shape.
    int img_width = 640;
    int img_height = 480;
};


class IrisTrackingManager : public ModelManager {
public:
    // TODO: init constructor with initialize
    IrisTrackingManager() {}
    ~IrisTrackingManager() {}

    void setup(int PORT) override;
    void update() override;
    void draw() override;

    // protobuf data blob.
    ::mediapipe::WrapperIris* wrapper;

    // point and rect used for drawing.
    std::vector<cv::Point3f> eye_left_pts;
    std::vector<cv::Point3f> eye_right_pts;
    std::vector<cv::Point3f> iris_left_pts;
    std::vector<cv::Point3f> iris_right_pts;

    // raw image's shape.
    int img_width = 640;
    int img_height = 480;

    // hand gesture
    typedef enum _iris_pose {
        SINGLE_FINGER = 0,
        DOUBLE_FINGER = 1,
        OTHERS        = 2
    }iris_pose;

    iris_pose pose;
};

class UpperBodyManager : public ModelManager {
public:
    // TODO: init constructor with initialize
    UpperBodyManager() {}
    ~UpperBodyManager() {}

    void setup(int PORT) override;
    void update() override;
    void draw() override;

    // protobuf data blob.
    ::mediapipe::WrapperUpperBody* wrapper;

    // point and rect used for drawing.
    std::vector<cv::Point3f> upper_body_pts;

    // raw image's shape.
    int img_width = 600;
    int img_height = 800;

    // hand gesture
    typedef enum _upper_body_pose {
        SINGLE_FINGER = 0,
        DOUBLE_FINGER = 1,
        OTHERS        = 2
    }upper_body_pose;

    upper_body_pose pose;
};

class ObjectTrackingManager : public ModelManager {
public:
    // TODO: init constructor with initialize
    ObjectTrackingManager() {}
    ~ObjectTrackingManager() {}

    void setup(int PORT) override;
    void update() override;
    void draw() override;

    // protobuf data blob.
    ::mediapipe::WrapperObjectTracking* wrapper;

    // struct for saving detection result
    typedef struct _LocationData {
        float x_min;
        float y_min;
        float width;
        float height;
    }LocationData;
    typedef struct _ObjectTrackedDetection {
        std::string label;
        float score;
        LocationData location_data;
    }ObjectTrackedDetection;
    std::vector<ObjectTrackedDetection> detection_result;
    int obj_num;
    // use for checking cell phone
    int phone_num{0};
    bool with_phone{false};
    cv::Point3f phone_center_pts;
    bool is_vertical{true};

    // raw image's shape.
    int img_width = 640;
    int img_height = 480;
};


#endif
