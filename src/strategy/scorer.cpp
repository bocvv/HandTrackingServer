#include "scorer.h"
#include "math.h"

#define PI 3.14159265

float compute_distance(cv::Point3f &v1, cv::Point3f &v2) {
    return sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
}

Scorer::Scorer(cv::Size img_size) {
    size_ = img_size;
    head_pose_estimator = PoseEstimator(img_size);
    shoulder_height[0] = size_.height;
    shoulder_height[1] = size_.height;
    eye_height[0] = size_.height;
    eye_height[1] = size_.height;
    last_neck_height = size_.height;
    min_neck_height = size_.height;
    min_nose_height = size_.height;
    normal_yaw = -45;
}

bool is_overlap(BoundingBox box1, BoundingBox box2) {
    return (min(box1.xmin + box1.width, box2.xmin + box2.width) > max(box1.xmin, box2.xmin))
            && (min(box1.ymin + box1.height, box2.ymin + box2.height) > max(box1.ymin, box2.ymin));
}

BehavioralClus Scorer::predict(FrameInfo & frame) {
    BehavioralClus clus;
    cout << "pose landmarks size: " << frame.pose_landmarks.size() << endl;

    // 根据pose判断是否有人
    if (frame.pose_landmarks.size() > 0) {
        // 预处理，在mediapipe下适用
        /*
        for (int i = 0; i < frame.pose_landmarks.size(); i++) {
            frame.pose_landmarks[i].x *= size_.width;
            frame.pose_landmarks[i].y *= size_.height;
        }*/

        // 判断肩膀倾斜，由于肩部关键点在侧面视角下的信息不可靠
        // 假如使用的话，需要尽可能在正面视角
        /*
        float shoulder_angle = atan2(abs(frame.pose_landmarks[6].y - frame.pose_landmarks[5].y),
                abs(frame.pose_landmarks[6].x - frame.pose_landmarks[5].x)) * 180 / PI;;
        if (abs(shoulder_angle) > 20) {
            clus.wrong_shoulder = true;
        }*/

        // 是否手在手机上
        vector<BoundingBox> hand_boxes;
        vector<BoundingBox> phone_boxes;
        for (int i = 0; i < frame.detections.size(); i++) {
            Detection* detection = frame.detections[i];
            BoundingBox box = detection->box;
            float angle = detection->angle;
            string object = detection->label;
            if (object == "cell phone") {
                phone_boxes.push_back(box);
            }
            if (object == "hand") {
                hand_boxes.push_back(box);
            }
        }
        bool play_phone = false;
        for (int hand_id = 0; hand_id < hand_boxes.size(); ++hand_id) {
            BoundingBox hand_box = hand_boxes[hand_id];
            for (int phone_id = 0; phone_id < phone_boxes.size(); ++phone_id) {
                BoundingBox phone_box = phone_boxes[phone_id];
                if (is_overlap(hand_box, phone_box)) {
                    play_phone = true;
                    break;
                }
            }
        }
        clus.play_phone = play_phone;
        if (clus.play_phone) {
            cout << "play phone" << endl;
        }

        // 根据人体关键点中鼻子和脖子之间的关系，判断抬头或者低头（大角度）
        // 此时大部分脸部内容不可见，估计的head pose信息不可靠
        float angle = atan2(frame.pose_landmarks[0].y - frame.pose_landmarks[18].y, frame.pose_landmarks[0].x - frame.pose_landmarks[18].x) * 180 / PI;
        bool head_down = false;
        if (angle > 10) {
            head_down = true;
        }
        //std::cout << frame.pose_landmarks[0].y << " " << frame.pose_landmarks[18].y << std::endl;
        //std::cout << "head down: " << head_down << std::endl;

        if (!head_down) { // 不是脸朝下
            if (frame.head_pose.size() == 3) {
                float yaw = frame.head_pose[0];
                float pitch = frame.head_pose[1];
                float roll = frame.head_pose[2];

                if (pitch > 10) { // 平视或者仰视
                    clus.look_away = true;
                }
            }

            // 更新头部高度，用于作为其他判断条件的参考值
            // 头部高度为头顶到颈部的距离
            head_height = sqrt((frame.pose_landmarks[17].x - frame.pose_landmarks[18].x) * (frame.pose_landmarks[17].x - frame.pose_landmarks[18].x)
                    + (frame.pose_landmarks[17].y - frame.pose_landmarks[18].y) * (frame.pose_landmarks[17].y - frame.pose_landmarks[18].y));

        }

        // 为了避免额外的因素干扰，坐姿检测只在检测到人脸的状况下生效
        if (frame.head_pose.size() == 0) { // 没有人脸信息
            clus.look_none = true;
            cout << "look none" << endl;
        }

        // 在脸部可见，且低头不严重的情况下，
        if (frame.head_pose.size() > 0 && !head_down && angle < 10) {
            cout << "yaw: " << frame.head_pose[0] << endl;
            if (yaw_list.size() > 40) {
                // 找到出现频率最高的角度
                // 以5度为bin
                vector<int> yaw_bin(37, 0);
                for (int i = 0; i < yaw_list.size(); i++) {
                    yaw_bin[yaw_list[i] / 5 + 18]++;
                }
                int max_num = yaw_bin[0];
                int max_index = 0;
                for (int i = 1; i < yaw_bin.size(); i++) {
                    if (yaw_bin[i] > max_num) {
                        max_num = yaw_bin[i];
                        max_index = i;
                    }
                }
                normal_yaw = max_index * 5 - 90;
                has_normal_yaw = true;
            } else {
                yaw_list.push_back(frame.head_pose[0]);
            }

            float yaw = 0;
            if (has_normal_yaw) {
                //cout << "normal yaw: " << normal_yaw << endl;
                yaw = frame.head_pose[0] - normal_yaw;
                //cout << "final yaw: " << yaw << endl;
            } else {
                yaw = frame.head_pose[0];
            }

            if (frame.head_pose[1] < -20) {
                // 低头情况下，yaw的估计不准确，不好判断方向
                // 默认在看书，除了手里有手机
                if (!clus.play_phone) {
                    clus.look_book = true;
                }
            } else {
                // 当yaw角度在（-10，10)范围时，表明用户在看台灯
                if (-10 < frame.head_pose[0] && frame.head_pose[0] < 10) {
                    // 由于低头时的yaw估计不准确，因此低头时不能判断看台灯
                    clus.look_lamp = true;
                    //cout << "look_lamp" << endl;
                } else {
                    HorizontalSight h_sight;
                    if (yaw < -15) { // 朝左看
                        cout << "look left" << endl;
                        h_sight = HorizontalSight::LEFT;
                    }
                    else if (yaw > 15) { // 朝右看
                        cout << "look right" << endl;
                        h_sight = HorizontalSight::RIGHT;
                    } else { // 朝中间看
                        cout << "look center" << endl;
                        h_sight = HorizontalSight::CENTER;
                    }

                    VerticalSight v_sight;
                    float pitch = frame.head_pose[1];
                    //cout << "pitch: " << pitch << endl;
                    if (pitch < -10) { // 低头
                        cout << "look down" << endl;
                        v_sight = VerticalSight::DOWN;
                    } else if (-10 <= pitch && pitch <= 10) { // 平视
                        //cout << "look horizon" << endl;
                        v_sight = VerticalSight::HORIZON;
                    } else { // 抬头
                        //cout << "look up" << endl;
                        v_sight = VerticalSight::UP;
                    }

                    if (frame.detections.size() > 0) {
                        // 存储不同方向的物体
                        // 分别对应，左侧，中间，右侧
                        vector<vector<string>> direction_objects(3, vector<string>());
                        for (int i = 0; i < frame.detections.size(); i++) {
                            Detection* detection = frame.detections[i];
                            BoundingBox box = detection->box;
                            float angle = detection->angle;
                            string object = detection->label;
                            if (object == "tv") { // 一般对应显示器
                                direction_objects[1].push_back("tv");
                            } else {
                                // 由于目前的角度会有一定的偏差，暂时先对台灯摆放位置所有约束
                                //angle -= normal_yaw;
                                if (angle > 20) {
                                    direction_objects[2].push_back(object);
                                } else if (angle < -20) {
                                    direction_objects[0].push_back(object);
                                } else {
                                    direction_objects[1].push_back(object);
                                }
                            }
                        }

                        vector<string> objects;
                        if (h_sight == HorizontalSight::LEFT) {
                            objects = direction_objects[0];
                        }
                        if (h_sight == HorizontalSight::CENTER) {
                            objects = direction_objects[1];
                        }
                        if (h_sight == HorizontalSight::RIGHT) {
                            objects = direction_objects[2];
                        }
                        //for (auto object : objects)
                        //    cout << object << endl;

                        // 当视线为中间时，需要根据垂直视线来判断是在看屏幕或者看书
                        if (h_sight == HorizontalSight::CENTER) {
                            if (v_sight == VerticalSight::DOWN) {
                                bool has_phone = false;
                                for (auto object : objects) {
                                    if (object == "cell phone")
                                        has_phone = true;
                                }
                                if (has_phone) {
                                    clus.look_phone = true;
                                } else {
                                    // 此时默认为看书状态
                                    clus.look_book = true;
                                }
                            } else {
                                // 此时，如果前方有显示器，则判断为看显示器
                                bool has_tv = false;
                                for (auto object : objects) {
                                    if (object == "tv")
                                        has_tv = true;
                                }
                                if (has_tv) {
                                    clus.look_monitor = true;
                                } else {
                                    clus.look_other = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (clus.look_phone) {
            cout << "look phone" << endl;
        }
        if (clus.look_lamp) {
            cout << "look lamp" << endl;
        }
        if (clus.look_book) {
            cout << "look book" << endl;
        }
        if (clus.look_monitor) {
            cout << "look monitor" << endl;
        }
        if (clus.look_other) {
            cout << "look other" << endl;
        }
        if (clus.look_none) {
            cout << "look none" << endl;
        }

        // 其次，坐姿检测只在相对静止的状态下才做判断，不在运动过程中进行判断
        // 目前根据颈部高度变化判断运动
        float nose_height = frame.pose_landmarks[0].y;
        float neck_height = frame.pose_landmarks[18].y;
        bool is_moving = false;
        if (abs(neck_height - last_neck_height) > 30) {
            is_moving = true;
        }
        // 更新颈部高度
        last_neck_height = neck_height;
        // 为了提高判断运动的准确率，会考虑最近几帧的状况，当几帧内超过一半的帧数在运动，判断为运动
        moving_info.push_back(is_moving);
        int num_moving = 4; // 初始化过程中均判断为运动
        if (moving_info.size() == 4) {
            num_moving = 1;
            for (auto item : moving_info) {
                if (item)
                    ++num_moving;
            }
            moving_info.erase(moving_info.begin());
        }
        if (num_moving >= 2) { // 当最近4帧有2帧处于运动时，则当前帧也判定为运动
            is_moving = true;
        }

        if (is_moving) {
            return clus;
        }

        if (neck_height < 200) { // 颈部高于一定值时，判定为站立状态
            return clus;
        }

        bool bad_pose = false;
        float nose_confidence = frame.pose_landmarks[0].z;
        float neck_confidence = frame.pose_landmarks[18].z;
        // 当鼻子的置信度大于一定阈值时，且高度在合理范围内时，更新最小高度
        if (nose_confidence > 0.6  && nose_height > 100) {
            if (nose_height < min_nose_height) {
                min_nose_height = nose_height;
            }
        }

        if (nose_height > min_nose_height + 150 || nose_height > min_nose_height + head_height) {
            bad_pose = true;
            clus.eye_near = true;
        }

        // 当颈部的置信度大于一定阈值时，且高度在合理范围内时，更新最小高度
        if (neck_confidence > 0.6) {
            if (neck_height < min_neck_height && neck_height > 200) {
                min_neck_height = neck_height;
            }
        }

        if (neck_height > min_neck_height + 100 || neck_height > min_neck_height + head_height) {
            bad_pose = true;
            clus.humpback = true;
        }

        cout << neck_confidence << " " << nose_confidence << endl;
        cout << "neck " << neck_height << " " << min_neck_height << endl;
        cout << "nose " << nose_height << " " << min_nose_height << endl;

        /*
        if (frame.face_landmarks.size() > 0) {
            // 在脸可见的情况下，更新肩部高度，避免用户站立
            // 且肩部比较正的情况下
            if (!clus.wrong_shoulder) {
                if (frame.pose_landmarks[11].y < shoulder_height[0] && frame.pose_landmarks[11].y > size_.height / 4.0) {
                    shoulder_height[0] = frame.pose_landmarks[11].y;
                }
                if (frame.pose_landmarks[12].y < shoulder_height[1] && frame.pose_landmarks[12].y > size_.height / 4.0) {
                    shoulder_height[1] = frame.pose_landmarks[12].y;
                }

                if (frame.pose_landmarks[11].y > shoulder_height[0] + 25
                    && frame.pose_landmarks[12].y > shoulder_height[1] + 25)
                    // 驼背
                    clus.humpback = true;
            }

            num_frames += 1;
            //head_pose_estimator.solve_pose(frame.face_landmarks);
            //cv::Vec3f angle = head_pose_estimator.get_angle();

            if (angle[0] > 0.2) { // 平视或者仰视
                clus.look_away = true;
            }

            if (angle[0] > -0.2 && angle[0] < 0.2) { // 平视状态
                head_height = head_pose_estimator.get_face_height();
            }

            //if (abs(angle[1]) * 180 / PI > 15) {
            //    clus.wrong_head = true;
            //}

            // 判断头左右倾斜
            // 采用两个眼睛的倾斜
            // 由于pose检测中的头部信息不准，转为用face信息判断
            //cv::Point2f left_eye((frame.face_landmarks[112].x + frame.face_landmarks[163].x) * 0.5,
            //                     (frame.face_landmarks[112].y + frame.face_landmarks[163].y) * 0.5);
            //cv::Point2f right_eye((frame.face_landmarks[359].x + frame.face_landmarks[381].x) * 0.5,
            //                     (frame.face_landmarks[359].y + frame.face_landmarks[381].y) * 0.5);
            //float roll_angle = atan2(abs(left_eye.y - right_eye.y), abs(left_eye.x - right_eye.x)) * 180 / PI;
            //if (abs(roll_angle) > 25) {
            //    clus.wrong_head = true;
            //}

            if (angle[0] < -0.1) { // 俯视状态，写作业
                if (left_eye.y < eye_height[0] && left_eye.y > size_.height / 4.0) {
                    eye_height[0] = left_eye.y;
                }

                if (right_eye.y < eye_height[1] && right_eye.y > size_.height / 4.0) {
                    eye_height[1] = right_eye.y;
                }

                if (left_eye.y > size_.height - 1.0 * head_height
                    && right_eye.y > size_.height - 1.0 * head_height) {
                    clus.eye_near = true;
                }

                if (left_eye.y > eye_height[0] + head_height / 2
                    && right_eye.y > eye_height[1] + head_height / 2) {
                    clus.eye_near = true;
                }
            }

            float left_right_angle = abs(angle[2]);
            if (left_right_angle >  PI / 2) {
                left_right_angle = PI - left_right_angle;
            }
            //if (left_right_angle * 180 / PI > 20) {
            //    clus.look_away = true;
            //}

            // 嘴巴张开
            float hori_mouth_dis = compute_distance(frame.face_landmarks[287], frame.face_landmarks[91]);
            float vert_mouth_dis = compute_distance(frame.face_landmarks[268], frame.face_landmarks[313]);
            if (vert_mouth_dis > 0.6 * hori_mouth_dis) {
                clus.yawning = true;
            }

            // 是否闭眼
            if (frame.iris_landmarks.size() > 0) {
                // 眼眶左边节点为0，右边节点为8
                // 眼眶上部节点为9-15
                // 眼眶下部节点为1-7
                // 以0-8连线为中心线，计算各点到中心点的垂直距离
                float alpha = (frame.iris_landmarks[0].y - frame.iris_landmarks[8].y) / (frame.iris_landmarks[0].x - frame.iris_landmarks[8].x);
                float beta = (frame.iris_landmarks[8].x * frame.iris_landmarks[0].y - frame.iris_landmarks[0].x * frame.iris_landmarks[8].y)
                        / (frame.iris_landmarks[0].x - frame.iris_landmarks[8].x);
                float total_dis = 0.0;
                for (int i = 3; i < 6; i++) {
                    float dis = (-alpha * frame.iris_landmarks[i].x + frame.iris_landmarks[i].y + beta) / sqrt(1 + alpha * alpha);
                    total_dis += abs(dis);
                }
                for (int i = 11; i < 14; i++) {
                    float dis = (-alpha * frame.iris_landmarks[i].x + frame.iris_landmarks[i].y + beta) / sqrt(1 + alpha * alpha);
                    total_dis += abs(dis);
                }
                float avg_dis = total_dis / 6;
                float eye_len = compute_distance(frame.iris_landmarks[0], frame.iris_landmarks[8]);
                bool left_close_eye = false;
                if (avg_dis <= eye_len * 0.05) {
                    left_close_eye = true;
                }
                int offset = 71;
                alpha = (frame.iris_landmarks[offset+0].y - frame.iris_landmarks[offset+8].y) / (frame.iris_landmarks[offset+0].x - frame.iris_landmarks[offset+8].x);
                beta = (frame.iris_landmarks[offset+8].x * frame.iris_landmarks[offset+0].y - frame.iris_landmarks[offset+0].x * frame.iris_landmarks[offset+8].y)
                        / (frame.iris_landmarks[offset+0].x - frame.iris_landmarks[offset+8].x);
                total_dis = 0.0;
                for (int i = 3; i < 6; i++) {
                    float dis = (-alpha * frame.iris_landmarks[i+offset].x + frame.iris_landmarks[i+offset].y + beta) / sqrt(1 + alpha * alpha);
                    total_dis += abs(dis);
                }
                for (int i = 11; i < 14; i++) {
                    float dis = (-alpha * frame.iris_landmarks[i+offset].x + frame.iris_landmarks[i+offset].y + beta) / sqrt(1 + alpha * alpha);
                    total_dis += abs(dis);
                }
                avg_dis = total_dis / 6;
                eye_len = compute_distance(frame.iris_landmarks[0], frame.iris_landmarks[8]);
                bool right_close_eye = false;
                if (avg_dis <= eye_len * 0.05) {
                    right_close_eye = true;
                }
                if (left_close_eye && right_close_eye) {
                    clus.close_eye = true;
                }
            } else {
                clus.close_eye = true;
            }
        } else {
            // 趴在桌子上
            clus.face_down = false;
        }*/
    } else {
        clus.presence = false;
    }
    return clus;
}

BehavioralClus Scorer::predict(NewFrameInfo & frame) {
    BehavioralClus clus;
    if (frame.action == ActionType::COMPUTER) {
        look_monitor = true;
    } else if (frame.action == ActionType::LEARN) {
        look_book = true;
    } else if (frame.action == ActionType::PHONE) {
        look_phone = true;
    }
    clus.learn_score = frame.learn_score;
    clus.good_pose_score = frame.good_pose_score;
    return clus;
}

AttentionState Scorer::score_attention(BehavioralClus & clus) {
    if (!clus.presence || clus.close_eye || clus.face_down) {
        return AttentionState::LOW;
    } else if (clus.look_away || clus.yawning) {
        return AttentionState::MEDIUM;
    } else {
        return AttentionState::HIGH;
    }
}

// AttentionState Scorer::score_attention(vector<BehavioralClus> & frames) {
    // // 专注度打分
    // int num_look_away = 0;
    // int num_yawning = 0;
    // int num_sleep = 0;
    // for (int i = 0; i < frames.size(); i++) {
        // if (frames[i].look_away) {
            // num_look_away++;
        // }
        // if (frames[i].yawning) {
            // num_yawning++;
        // }
        // if (frames[i].close_eye || frames[i].face_down) {
            // num_sleep++;
        // }
    // }
    // if (float(num_sleep) / frames.size() > 0.4
            // || float(num_look_away) / frames.size() > 0.4) {
        // return AttentionState::LOW;
    // }
    // if (float(num_sleep) / frames.size() > 0.1
            // || float(num_yawning) / frames.size() > 0.1
            // || float(num_look_away) / frames.size() > 0.2) {
        // return AttentionState::MEDIUM;
    // }
    // return AttentionState::HIGH;
// }

// HealthState Scorer::score_health(vector<BehavioralClus> & frames) {
    // // 健康度打分
    // int num_wrong_head = 0;
    // int num_wrong_shoulder = 0;
    // int num_humpback = 0;
    // int num_eye_near = 0;
    // for (int i = 0; i < frames.size(); i++) {
        // if (frames[i].wrong_head) {
            // num_wrong_head++;
        // }
        // //if (frames[i].wrong_shoulder) {
        // //    num_wrong_shoulder++;
        // //}
        // if (frames[i].humpback) {
            // num_humpback++;
        // }
        // if (frames[i].eye_near) {
            // num_eye_near++;
        // }
    // }
    // if (float(num_wrong_head) / frames.size() > 0.8
            // || float(num_wrong_shoulder) / frames.size() > 0.8
            // || float(num_humpback) / frames.size() > 0.8
            // || float(num_eye_near) / frames.size() > 0.8) {
        // return HealthState::LOW;
    // }
    // if (float(num_wrong_head) / frames.size() > 0.3
            // || float(num_wrong_shoulder) / frames.size() > 0.3
            // || float(num_humpback) / frames.size() > 0.3
            // || float(num_eye_near) / frames.size() > 0.333) {
        // return HealthState::MEDIUM;
    // }
    // return HealthState::HIGH;
// }

AttentionState Scorer::score_attention(vector<BehavioralClus> & frames) {
    // float computer_score = 0.0;
    float learn_score = 0.0;
    // float phone_score = 0.0;
    for (int i = 0; i < frames.size(); i++) {
        learn_score += frames[i].learn_score;
        // if (frames[i].action == ActionType::COMPUTER) {
            // computer_score += frames[i].action_score;
        // } else if (frames[i].action == ActionType::LEARN) {
            // learn_score += frames[i].action_score;
        // } else if (frames[i].action == ActionType::PHONE) {
            // phone_score += frames[i].action_score;
        // }
    }

    learn_score /= float(frames.size());
    if (learn_score > 0.75) {
        return AttentionState::HIGH;
    } else if (learn_score < 0.1) {
        return AttentionState::LOW;
    }
    return AttentionState::MEDIUM;
}

HealthState Scorer::score_health(vector<BehavioralClus> & frames) {
    float good_pose_score = 0.0;
    for (int i = 0; i < frames.size(); i++) {
        // if (frames[i].pose == PoseType::UPRIGHT) {
            // good_pose_score += frames[i].pose_score;
        // }
        good_pose_score += frames[i].good_pose_score;
    }
    good_pose_score /= float(frames.size());
    if (good_pose_score > 0.75) {
        return HealthState::HIGH;
    } else if (good_pose_score < 0.1) {
        return HealthState::LOW;
    }
    return HealthState::MEDIUM;
}

AttentionState Scorer::count_attention(vector<AttentionState> & states) {
    float total_score = 0.0;
    for (auto state : states) {
        total_score += static_cast<int>(state);
    }
    float avg_score = total_score / states.size();
    if (avg_score >= 2.5) {
        return AttentionState::HIGH;
    } else if (1.5 < avg_score < 2.5) {
        return AttentionState::MEDIUM;
    } else {
        return AttentionState::LOW;
    }
}

HealthState Scorer::count_health(vector<HealthState> & states) {
    float total_score = 0.0;
    for (auto state : states) {
        //total_score += state;
        total_score += static_cast<int>(state);
    }
    float avg_score = total_score / states.size();
    if (avg_score >= 2.5) {
        return HealthState::HIGH;
    } else if (1.5 < avg_score < 2.5) {
        return HealthState::MEDIUM;
    } else {
        return HealthState::LOW;
    }
}
