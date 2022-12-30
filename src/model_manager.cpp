/*************************************************************************
    > File Name: model_manager.cpp
    > Author: Bowei Wang
    > Mail: wangbw@rd.neteasy.com
    > Created Time: 五  8/28 18:05:03 2020
 ************************************************************************/

#include <iostream>
#include <sstream>
#include <model_manager.h>


using namespace std;


ModelManager::~ModelManager() {}

void joint_node(cv::Mat raw_image, std::vector<cv::Point3f> & hand_pts, \
                int idx0, int idx1, cv::Scalar color)
{
    // draw line between two nodes
    cv::Point node_0(hand_pts[idx0].x, hand_pts[idx0].y);
    cv::Point node_1(hand_pts[idx1].x, hand_pts[idx1].y);
    cv::line(raw_image, node_0, node_1, color, 2);
}

float fVec3f_dist(std::vector<cv::Point3f> & hand_pts, int idx0, int idx1)
{
    float delta_x = hand_pts[idx0].x;
    float delta_y = hand_pts[idx0].y;
    delta_x -= hand_pts[idx1].x;
    delta_y -= hand_pts[idx1].y;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

bool is_curved(std::vector<cv::Point3f> & hand_pts, int btm, int top)
{
    // compute the distance between finger top and bottom.
    ModelManager::fVec3f vector_top_btm = {
        .x = hand_pts[btm].x - hand_pts[top].x,
        .y = hand_pts[btm].y - hand_pts[top].y,
        .z = hand_pts[btm].z - hand_pts[top].z
    };

    ModelManager::fVec3f vector_top_btm_next = {
        .x = hand_pts[btm + 1].x - hand_pts[top].x,
        .y = hand_pts[btm + 1].y - hand_pts[top].y,
        .z = hand_pts[btm + 1].z - hand_pts[top].z
    };
    float vector_dot = vector_top_btm.x * vector_top_btm_next.x + vector_top_btm.y * \
                vector_top_btm_next.y; //+ vector_top_btm.z * vector_top_btm_next.z;
    // compute the distance between finger top and the next point of bottom(+1).
    //float dist_btm_next_top = fVec3f_dist(hand_pts, top, btm + 1);

    if (vector_dot < 0) {
        return true;
    }

    return false;
}

void HandManager::setup(int PORT) {
    // create the socket and bind to porti[default= 8080].
    std::string bind_addr = "127.0.0.1";
    ofxUDPSettings settings;
    settings.receiveOn(bind_addr, PORT);
    settings.blocking = true;

    udp_mgr.Setup(settings);

    // initialize wrapper proto object
    wrapper = new ::mediapipe::WrapperHandTracking();
    wrapper->InitAsDefaultInstance();

    // create an empty hand_pts list with the 21 hand points
    cv::Point3f point_3d(0.f, 0.f, 0.f);
    for (int i=0; i<21; i++)
        hand_pts.emplace_back(point_3d);
}

void HandManager::update() {
    // check for incoming messages
    size_n = udp_mgr.Receive(udp_message, 100000);
    if (size_n > 0){
        // accept the incoming proto
        wrapper->Clear();
        wrapper->ParseFromArray(udp_message, udp_mgr.GetReceiveBufferSize());
        
        // update the hand_pts list
        if (wrapper->landmarks().landmark_size() > 1) {
            hand_num = 1;
        } else {
            hand_num = 0;
        }

        for (int i = 0; i < wrapper->landmarks().landmark_size(); i++) {
            auto& landmark = wrapper->landmarks().landmark(i);
            hand_pts[i].x = landmark.x();
            hand_pts[i].y = landmark.y();
            hand_pts[i].z = landmark.z();
            // cout << "Landmark " << i << ": " << endl;
            // cout << landmark.DebugString() << endl;
        }
        hand_center_pts.push_back(hand_pts[8]);
        // cout << wrapper->landmarks().DebugString() << endl;

        // update the hand rectangle
        if (wrapper->rect().x_center() != 0 && wrapper->rect().y_center() != 0) {
            hand_rect.hand_cx = wrapper->rect().x_center();
            hand_rect.hand_cy = wrapper->rect().y_center();
            hand_rect.hand_w = wrapper->rect().width();
            hand_rect.hand_h = wrapper->rect().height();
            hand_rect.rotation = wrapper->rect().rotation();
            // cout << "rect: " <<  endl;
            // cout << wrapper->rect().DebugString() << endl;
        }
    }
}

void HandManager::draw() {
    // drawing landmarks at the white image
    white_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(206, 206, 205));

    if (hand_num == 0) {
        return;
    }

    // Draw and Label the Hand Points
    cv::Scalar col_red(255, 0, 0);
    cv::Scalar col_yellow(0, 0, 0);
    cv::Scalar col_green(0, 255, 0);
    cv::Scalar col_cyan(0, 255, 255);
    cv::Scalar col_violet(255, 0, 255);
    cv::Scalar col_palm(204, 204, 204);
    cv::Scalar col_node(172, 172, 172);

    cv::Scalar colj;
    cv::Scalar coln = col_node;
    cv::Scalar colp = col_palm;

    for (int i = 0; i < hand_pts.size(); ++i) {
        if (i >= 17)
            colj = col_violet;
        else if (i >= 13)
            colj = col_cyan;
        else if (i >= 9)
            colj = col_green;
        else if (i >= 5)
            colj = col_yellow;
        else
            colj = col_red;
        
        cv::Point p(hand_pts[i].x, hand_pts[i].y);
        cv::circle(white_image, p, 4, colj, -1);
    }

    // joint node.
    joint_node(white_image, hand_pts, 0, 1, coln);
    joint_node(white_image, hand_pts, 0, 17, coln);
    joint_node(white_image, hand_pts, 1, 5, coln);
    joint_node(white_image, hand_pts, 5, 9, coln);
    joint_node(white_image, hand_pts, 9, 13, coln);
    joint_node(white_image, hand_pts, 13, 17, coln);
    for (int i = 0; i < 5; ++i) {
        int idx0 = 4 * i + 1;
        int idx1 = idx0 + 1;
        joint_node(white_image, hand_pts, idx0, idx1, coln);
        joint_node(white_image, hand_pts, idx0 + 1, idx1 + 1, coln);
        joint_node(white_image, hand_pts, idx0 + 2, idx1 + 2, coln);
    }
}

void HandManager::infer_gesture() {
    if (size_n <= 0) {
        return;
    }
    std::stringstream gesture_ss;

    finger_pose = OTHERS;
    bool is_forefinger_curve = is_curved(hand_pts, 5, 8);
    bool is_midfinger_curve = is_curved(hand_pts, 9, 12);
    bool is_ringfinger_curve = is_curved(hand_pts, 13,16);
    bool is_littlefinger_curve = is_curved(hand_pts, 17, 20);
    if (!is_forefinger_curve) {
        if (is_ringfinger_curve || is_littlefinger_curve) {
            if (is_midfinger_curve) {
                finger_pose = SINGLE_FINGER;
            } else {
                finger_pose = DOUBLE_FINGER;
            }
        }
    }

    switch (finger_pose)
    {
        case SINGLE_FINGER:
            gesture_ss << "gesture = 1";
            break;
        case DOUBLE_FINGER:
            gesture_ss << "gesture = 2";
            break;
        default:
            gesture_ss << "gesture = others";
            break;
    }

    cv::putText(white_image, gesture_ss.str(), cv::Point(16, 32), \
                cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(255, 0, 0));
}


void MultiHandManager::setup(int PORT) {
    // create the socket and bind to porti[default= 8080].
    ofxUDPSettings settings;
    settings.receiveOn(PORT);
    settings.blocking = true;

    udp_mgr.Setup(settings);

    // initialize wrapper proto object
    wrapper = new ::mediapipe::WrapperMultiHand();
    wrapper->InitAsDefaultInstance();

    // create an empty hand_pts list with the 21 hand points
    cv::Point3f point_3d(0.f, 0.f, 0.f);
    for (int j = 0; j < HAND_POINT_NUMBER; j++)
        hand_pts.emplace_back(point_3d);
    for (int i = 0; i < MAX_HAND_NUMBER; i++)
        hand_pts_group.push_back(hand_pts);
}

void MultiHandManager::update() {
    size_n = udp_mgr.Receive(udp_message, 100000);
    // check for incoming messages
    if (size_n > 0) {
        // accept the incoming proto
        wrapper->Clear();
        wrapper->ParseFromArray(udp_message, udp_mgr.GetReceiveBufferSize());
        
        // update the _pts list
        hand_num = wrapper->landmarkgroup().landmarklist_size();
        for (int i = 0; i < hand_num; i++) {
            auto& landmarklist = wrapper->landmarkgroup().landmarklist(i);
            for (int j = 0; j < landmarklist.landmark_size(); j++) {
                auto& landmark = landmarklist.landmark(j);
                hand_pts_group[i][j].x = landmark.x();
                hand_pts_group[i][j].y = landmark.y();
                hand_pts_group[i][j].z = landmark.z();
                // cout << "Landmark " << j << ": " << endl;
                // cout << landmark.DebugString() << endl;
            }

            hand_center_pts.push_back(hand_pts_group[i][12]);
        }

        // update the hand rectangle
        /*
        if (wrapper->rect().x_center() != 0 && wrapper->rect().y_center() != 0) {
            hand_rect.hand_cx = wrapper->rect().x_center() * img_width;
            hand_rect.hand_cy = wrapper->rect().y_center() * img_height;
            hand_rect.hand_w = wrapper->rect().width() * img_width;
            hand_rect.hand_h = wrapper->rect().height() * img_height;
            hand_rect.rotation = wrapper->rect().rotation();
            cout << "rect: " <<  endl;
            cout << wrapper->rect().DebugString() << endl;
        }
        */
    }
}

void MultiHandManager::draw() {
    // drawing landmarks at the white image
    white_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(206, 206, 205));

    if (hand_num == 0 || size_n <= 0) {
        return;
    }

    // Draw and Label the hand Points
    cv::Scalar col_red(255, 0, 0);
    cv::Scalar col_green(0, 255, 0);

    for (int i = 0; i < hand_num; ++i) {
        for (int j = 0; j < HAND_POINT_NUMBER; ++j) {
            cv::Point p(hand_pts_group[i][j].x, hand_pts_group[i][j].y);
            cv::circle(white_image, p, 2, col_red, -1);
            // joint node.
            // joint_node(white_image, hand_pts, 0, 1, coln);
        }
    }
}


void FaceManager::setup(int PORT) {
    // create the socket and bind to porti[default= 8080].
    ofxUDPSettings settings;
    settings.receiveOn(PORT);
    settings.blocking = true;

    udp_mgr.Setup(settings);

    // initialize wrapper proto object
    wrapper = new ::mediapipe::WrapperFaceMesh();
    wrapper->InitAsDefaultInstance();

    // create an empty face_pts list with the 21 face points
    cv::Point3f point_3d(0.f, 0.f, 0.f);
    for (int j = 0; j < FACE_POINT_NUMBER; j++)
        face_pts.emplace_back(point_3d);
    for (int i = 0; i < MAX_FACE_NUMBER; i++)
        face_pts_group.push_back(face_pts);
}

void FaceManager::update() {
    size_n = udp_mgr.Receive(udp_message, 100000);
    // check for incoming messages
    if (size_n > 0) {
        // accept the incoming proto
        wrapper->Clear();
        wrapper->ParseFromArray(udp_message, udp_mgr.GetReceiveBufferSize());
        
        // update the _pts list
        face_num = wrapper->landmarkgroup().landmarklist_size();
        for (int i = 0; i < face_num; i++) {
            auto& landmarklist = wrapper->landmarkgroup().landmarklist(i);
            for (int j = 0; j < landmarklist.landmark_size(); j++) {
                auto& landmark = landmarklist.landmark(j);
                face_pts_group[i][j].x = landmark.x();
                face_pts_group[i][j].y = landmark.y();
                face_pts_group[i][j].z = landmark.z();
                // cout << "Landmark " << j << ": " << endl;
                // cout << landmark.DebugString() << endl;
            }
        }

        // update the face rectangle
        /*
        if (wrapper->rect().x_center() != 0 && wrapper->rect().y_center() != 0) {
            face_rect.face_cx = wrapper->rect().x_center() * img_width;
            face_rect.face_cy = wrapper->rect().y_center() * img_height;
            face_rect.face_w = wrapper->rect().width() * img_width;
            face_rect.face_h = wrapper->rect().height() * img_height;
            face_rect.rotation = wrapper->rect().rotation();
            cout << "rect: " <<  endl;
            cout << wrapper->rect().DebugString() << endl;
        }
        */
    } else {
        for (int j = 0; j < FACE_POINT_NUMBER; j++) {
            face_pts_group[0][j].x = 0;
            face_pts_group[0][j].y = 0;
            face_pts_group[0][j].z = 0;
        }
    }
}

void FaceManager::draw() {
    // drawing landmarks at the white image
    white_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(206, 206, 205));

    if (face_num == 0 || size_n <= 0) {
        return;
    }

    // Draw and Label the face Points
    cv::Scalar col_red(255, 0, 0);
    cv::Scalar col_green(0, 255, 0);

    for (int i = 0; i < face_num; ++i) {
        for (int j = 0; j < FACE_POINT_NUMBER; ++j) {
            cv::Point p(face_pts_group[i][j].x, face_pts_group[i][j].y);
            cv::circle(white_image, p, 2, col_red, -1);
            // joint node.
            // joint_node(white_image, face_pts, 0, 1, coln);
        }
    }
}


void IrisTrackingManager::setup(int PORT) {
    // create the socket and bind to porti[default= 8080].
    ofxUDPSettings settings;
    settings.receiveOn(PORT);
    settings.blocking = true;

    udp_mgr.Setup(settings);

    // initialize wrapper proto object
    wrapper = new ::mediapipe::WrapperIris();
    wrapper->InitAsDefaultInstance();

    // create an empty iris_pts list with the 21 iris points
    cv::Point3f point_3d(0.f, 0.f, 0.f);
    for (int i = 0; i < EYE_CONTOUR_POINT_NUM; i++)
        eye_left_pts.emplace_back(point_3d);
    for (int i = 0; i < EYE_CONTOUR_POINT_NUM; i++)
        eye_right_pts.emplace_back(point_3d);
    for (int i = 0; i < IRIS_POINT_NUM; i++)
        iris_left_pts.emplace_back(point_3d);
    for (int i = 0; i < IRIS_POINT_NUM; i++)
        iris_right_pts.emplace_back(point_3d);
}

void IrisTrackingManager::update() {
    // check for incoming messages
    size_n = udp_mgr.Receive(udp_message,100000);
    // cout << "recieve meg successful" << endl;
    if (size_n > 0) {
        // accept the incoming proto
        wrapper->Clear();
        wrapper->ParseFromArray(udp_message, udp_mgr.GetReceiveBufferSize());
        
        // update the iris_pts list
        for (int i = 0; i < wrapper->eye_landmarks_left().landmark_size(); i++) {
            auto& landmark = wrapper->eye_landmarks_left().landmark(i);
            eye_left_pts[i].x = landmark.x();
            eye_left_pts[i].y = landmark.y();
            eye_left_pts[i].z = landmark.z();
            // cout << "Landmark " << i << ": " << endl;
            // cout << landmark.DebugString() << endl;
        }
        for (int i = 0; i < wrapper->eye_landmarks_right().landmark_size(); i++) {
            auto& landmark = wrapper->eye_landmarks_right().landmark(i);
            eye_right_pts[i].x = landmark.x();
            eye_right_pts[i].y = landmark.y();
            eye_right_pts[i].z = landmark.z();
            // cout << "Landmark " << i << ": " << endl;
            // cout << landmark.DebugString() << endl;
        }
        for (int i = 0; i < wrapper->iris_landmarks_left().landmark_size(); i++) {
            auto& landmark = wrapper->iris_landmarks_left().landmark(i);
            iris_left_pts[i].x = landmark.x();
            iris_left_pts[i].y = landmark.y();
            iris_left_pts[i].z = landmark.z();
            // cout << "Landmark " << i << ": " << endl;
            // cout << landmark.DebugString() << endl;
        }
        for (int i = 0; i < wrapper->iris_landmarks_right().landmark_size(); i++) {
            auto& landmark = wrapper->iris_landmarks_right().landmark(i);
            iris_right_pts[i].x = landmark.x();
            iris_right_pts[i].y = landmark.y();
            iris_right_pts[i].z = landmark.z();
            // cout << "Landmark " << i << ": " << endl;
            // cout << landmark.DebugString() << endl;
        }
    } else {
        for (int i = 0; i < EYE_CONTOUR_POINT_NUM; i++) {
            eye_left_pts[i].x = 0;
            eye_left_pts[i].y = 0;
            eye_left_pts[i].z = 0;
        }
        for (int i = 0; i < EYE_CONTOUR_POINT_NUM; i++) {
            eye_right_pts[i].x = 0;
            eye_right_pts[i].y = 0;
            eye_right_pts[i].z = 0;
        }
        for (int i = 0; i < IRIS_POINT_NUM; i++) {
            iris_left_pts[i].x = 0;
            iris_left_pts[i].y = 0;
            iris_left_pts[i].z = 0;
        }
        for (int i = 0; i < IRIS_POINT_NUM; i++) {
            iris_right_pts[i].x = 0;
            iris_right_pts[i].y = 0;
            iris_right_pts[i].z = 0;
        }
    }
}

void IrisTrackingManager::draw() {
    // drawing landmarks at the white image
    white_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(206, 206, 205));

    if (size_n <= 0) {
        return;
    }

    // Draw and Label the iris Points
    cv::Scalar col_red(255, 0, 0);
    cv::Scalar col_yellow(0, 0, 0);
    cv::Scalar col_green(0, 255, 0);
    cv::Scalar col_cyan(0, 255, 255);
    cv::Scalar col_violet(255, 0, 255);
    cv::Scalar col_palm(204, 204, 204);
    cv::Scalar col_node(172, 172, 172);

    cv::Scalar colj;
    cv::Scalar coln = col_node;
    cv::Scalar colp = col_palm;

    for (int i = 0; i < EYE_CONTOUR_POINT_NUM; ++i) {
        if (i >= 17)
            colj = col_violet;
        else if (i >= 13)
            colj = col_cyan;
        else if (i >= 9)
            colj = col_green;
        else if (i >= 5)
            colj = col_yellow;
        else
            colj = col_red;
        
        cv::Point p_left(eye_left_pts[i].x, eye_left_pts[i].y);
        cv::Point p_right(eye_right_pts[i].x, eye_right_pts[i].y);
        cv::circle(white_image, p_left, 2, colj, -1);
        cv::circle(white_image, p_right, 2, colj, -1);
    }
    for (int i = 0; i < IRIS_POINT_NUM; ++i) {
        if (i >= 17)
            colj = col_violet;
        else if (i >= 13)
            colj = col_cyan;
        else if (i >= 9)
            colj = col_green;
        else if (i >= 5)
            colj = col_yellow;
        else
            colj = col_red;
        
        cv::Point p_left(iris_left_pts[i].x, iris_left_pts[i].y);
        cv::Point p_right(iris_right_pts[i].x, iris_right_pts[i].y);
        cv::circle(white_image, p_left, 2, colj, -1);
        cv::circle(white_image, p_right, 2, colj, -1);
    }
}


void UpperBodyManager::setup(int PORT) {
    // create the socket and bind to porti[default= 8080].
    ofxUDPSettings settings;
    settings.receiveOn(PORT);
    settings.blocking = true;

    udp_mgr.Setup(settings);

    // initialize wrapper proto object
    wrapper = new ::mediapipe::WrapperUpperBody();
    wrapper->InitAsDefaultInstance();

    // create an empty upper_body_pts list with the 21 upper_body points
    cv::Point3f point_3d(0.f, 0.f, 0.f);
    for (int i = 0; i < UPPER_BODY_POINT_NUM; i++)
        upper_body_pts.emplace_back(point_3d);
}

void UpperBodyManager::update() {
    // check for incoming messages
    size_n = udp_mgr.Receive(udp_message,100000);
    if (size_n > 0){
        // accept the incoming proto
        wrapper->Clear();
        wrapper->ParseFromArray(udp_message, udp_mgr.GetReceiveBufferSize());
        
        // update the upper_body_pts list
        for (int i = 0; i < UPPER_BODY_POINT_NUM; i++) {
            auto& landmark = wrapper->landmarks().landmark(i);
            upper_body_pts[i].x = landmark.x();
            upper_body_pts[i].y = landmark.y();
            upper_body_pts[i].z = landmark.z();
            // cout << "Landmark " << i << ": " << endl;
            // cout << landmark.DebugString() << endl;
        }
    } else {
        for (int i = 0; i < UPPER_BODY_POINT_NUM; i++) {
            upper_body_pts[i].x = 0;
            upper_body_pts[i].y = 0;
            upper_body_pts[i].z = 0;
        }
    }
}

void UpperBodyManager::draw() {
    // drawing landmarks at the white image
    // white_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(206, 206, 205));

    // Draw and Label the upper_body Points
    cv::Scalar col_red(255, 0, 0);
    cv::Scalar col_yellow(0, 0, 0);
    cv::Scalar col_green(0, 255, 0);
    cv::Scalar col_cyan(0, 255, 255);
    cv::Scalar col_violet(255, 0, 255);
    cv::Scalar col_palm(204, 204, 204);
    cv::Scalar col_node(172, 172, 172);

    cv::Scalar colj;
    cv::Scalar coln = col_node;
    cv::Scalar colp = col_palm;

    for (int i = 0; i < upper_body_pts.size(); ++i) {
        if (i >= 17)
            colj = col_violet;
        else if (i >= 13)
            colj = col_cyan;
        else if (i >= 9)
            colj = col_green;
        else if (i >= 5)
            colj = col_yellow;
        else
            colj = col_red;
        
        cv::Point p(upper_body_pts[i].x, upper_body_pts[i].y);
        cv::circle(white_image, p, 4, colj, -1);
    }
}

void ObjectTrackingManager::setup(int PORT) {
    // create the socket and bind to porti[default= 8080].
    ofxUDPSettings settings;
    settings.receiveOn(PORT);
    settings.blocking = true;

    udp_mgr.Setup(settings);

    // initialize wrapper proto object
    wrapper = new ::mediapipe::WrapperObjectTracking();
    wrapper->InitAsDefaultInstance();
}

void ObjectTrackingManager::update() {
    size_n = udp_mgr.Receive(udp_message, 100000);
    // check for incoming messages
    if (size_n > 0) {
        // accept the incoming proto
        wrapper->Clear();
        wrapper->ParseFromArray(udp_message, udp_mgr.GetReceiveBufferSize());
        
        // update the detection result
        obj_num = wrapper->detections().detection_size();
        detection_result.clear();
        for (int i = 0; i < obj_num; i++) {
            ObjectTrackedDetection tracked_detection;
            auto& detection = wrapper->detections().detection(i);
            tracked_detection.label = detection.label(0);
            tracked_detection.score = detection.score(0);
            tracked_detection.location_data.x_min = detection.location_data().relative_bounding_box().xmin();
            tracked_detection.location_data.y_min = detection.location_data().relative_bounding_box().ymin();
            tracked_detection.location_data.width = detection.location_data().relative_bounding_box().width();
            tracked_detection.location_data.height= detection.location_data().relative_bounding_box().height();
            detection_result.push_back(tracked_detection);

            if (tracked_detection.label == "cell phone") {
                with_phone = true;
                phone_num++;
                phone_center_pts.x = tracked_detection.location_data.x_min + tracked_detection.location_data.width / 2;
                phone_center_pts.y = tracked_detection.location_data.y_min + tracked_detection.location_data.height / 2;
                phone_center_pts.z = 0;
            }
            // cout << "Detection_" << i << ": " << endl;
            // cout << detection.DebugString() << endl;
        }
    }
}

void ObjectTrackingManager::draw() {
    // drawing landmarks at the white image
    white_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(206, 206, 205));

    if (obj_num == 0 || size_n <= 0) {
        return;
    }

    // Draw and Label the face Points
    cv::Scalar col_red(255, 0, 0);
    cv::Scalar col_green(0, 255, 0);

    for (int i = 0; i < obj_num; ++i) {
        // joint node.
        // joint_node(white_image, face_pts, 0, 1, coln);
    }
}
