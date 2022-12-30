#ifndef ALADIN_SCORER_
#define ALADIN_SCORER_

#include <vector>
#include <opencv2/opencv.hpp>

#include "pose_estimator.h"

using namespace std;

// 有最大长度的队列
template <typename T, int MaxLen, typename Container=std::deque<T>>
class FixedQueue : public std::queue<T, Container> {
public:
    void push(const T& value) {
        if (this->size() == MaxLen) {
           this->c.pop_front();
        }
        std::queue<T, Container>::push(value);
    }
};

// 检测的boundingbox信息
struct BoundingBox {
    int xmin;
    int ymin;
    int width;
    int height;
};

// 检测结果
struct Detection {
    // 物体的类别
    string label;
    // 物体相对于人的角度
    float angle;
    // 物体的包围框
    BoundingBox box;
    // 检测的置信度
    float score;
};

struct FrameInfo {

    int frame_id;

    // 脸部pose信息
    vector<float> head_pose;

    // 脸部关键点信息
    vector<cv::Point3f> face_landmarks;

    // 身体关键点信息
    vector<cv::Point3f> pose_landmarks;

    // 虹膜关键点信息
    vector<cv::Point3f> iris_landmarks;

    // 物体检测信息
    vector<Detection*> detections;

    bool has_multiple_humans;
};

struct BehavioralClus {
    float learn_score = 0.0;
    float good_pose_score = 0.0;

    //=================================================
    // 是否在镜头里
    // 用人体pose检测判断
    bool presence = true;

    /// 专注相关

    // 转移目光，主要是通过头的pose判断
    bool look_away = false;

    // 东张西望，依靠一段时间的pose变化
    cv::Point3f head_pose;

    // 是否张开嘴
    bool mouth_open = false;

    // 是否在打哈欠，通过嘴的张开，需要持续一定时间(1.5s)
    bool yawning = false;

    // 是否闭眼，通过眼睛处的关键点
    // 必要时，结合irir关键点信息
    bool close_eye = false;


    // 学习时间统计相关
    // 是否在玩手机
    bool play_phone = false;
    bool look_phone = false;

    bool look_lamp = false;

    bool look_monitor = false;

    bool look_book = false;

    bool look_other = false;

    bool look_none = false;

    // 托腮，通过手与脸部的关系
    // 需要考虑深度信息

    /// 姿势相关

    // 肩膀倾斜
    bool wrong_shoulder = false;

    // 头左右倾斜
    bool wrong_head = false;

    // 是否弯腰
    bool humpback = false;

    // 眼睛离桌面太近
    bool eye_near = false;

    /// 姿势和专注都相关

    // 趴在桌子上
    // 利用pose检测和脸部检测结合
    bool face_down = false;

    string to_string() {
        string str;
	    str += "presence";
        if (presence) {
            //str += " true;";
            str += " faced_down";
            if (face_down) {
                str += " true;";
            } else {
                //str += " false;";
		        str += " close_eye";
		        if (close_eye) {
                    str += " true;";
		        } else {
                    //str += " false;";
                }
            }
            if (wrong_shoulder) {
                str += " wrong_shoulder";
		    } else {
                //str += " false;";
            }
            if (wrong_head) {
                str += " wrong_head";
		    } else {
                //str += " false;";
            }
	    } else {
            str += " false;";
        }
        return str;
    }
};

// 动作类别
enum class ActionType {
    COMPUTER = 1, // "computer"
    LEARN = 2, // "learn"
    PHONE = 3 // "phone"
};
// 坐姿类别
enum class ActionType {
    BAD = 1,
    UPRIGHT = 2
};

// struct NewBehavioralClus {
    // ActionType action;
    // PoseType pose;
    // // float action_score = 0.0;
    // float learn_score = 0.0;
    // float good_pose_score = 0.0;
// };

struct NewFrameInfo {
    ActionType action;
    PoseType pose;
    // float action_score = 0.0;
    float learn_score = 0.0;
    float good_pose_score = 0.0;
};

// 水平方向的视线
enum class HorizontalSight {
    LEFT = 1,
    CENTER = 2,
    RIGHT = 3
};

enum class VerticalSight {
    UP = 1,
    HORIZON = 2,
    DOWN = 3
};

// 专注程度
enum class AttentionState {
    HIGH = 3,          // 专注项都为false
    MEDIUM = 2,        // look_away, yawning
    LOW = 1           // close_eye, face_down. not present
};

// 健康度
enum class HealthState {
    HIGH = 3,
    MEDIUM = 2,
    LOW = 1
};

class Scorer {
public:

    Scorer(cv::Size img_size);

    BehavioralClus predict(FrameInfo & frame);
    BehavioralClus predict(NewFrameInfo & frame);

    AttentionState score_attention(BehavioralClus & clus);

    // 以分钟为单位进行请求
    AttentionState score_attention(vector<BehavioralClus> & frames);

    HealthState score_health(vector<BehavioralClus> & frames);

    // // 以分钟为单位进行请求
    // AttentionState score_attention(vector<NewBehavioralClus> & frames);

    // HealthState score_health(vector<NewBehavioralClus> & frames);

    // 计算总得分
    AttentionState count_attention(vector<AttentionState> & states);

    HealthState count_health(vector<HealthState> & states);

private:

    PoseEstimator head_pose_estimator;

    // 记录正常肩膀高度
    cv::Vec2f shoulder_height;

    // 记录头的高度，用来作为参考
    float head_height;

    // 记录正常的眼睛高度
    cv::Vec2f eye_height;

    // 记录正常的颈部高度，相比于肩部，更加稳定
    float min_neck_height;

    // 记录上一帧的颈部高度，用于判断运动
    float last_neck_height;

    // 记录正常的鼻子高度，相比于眼睛，更加稳定
    float min_nose_height;

    // 记录人的正常yaw角度
    float normal_yaw;

    bool has_normal_yaw = false;

    // 为了获得台灯与人的角度，采用统计的办法，因此需要记录一段时间的角度信息
    //FixedQueue<float, 100> yaw_list;
    vector<float> yaw_list;

    vector<bool> moving_info;

    cv::Size size_;

    int num_frames = 0;
};

#endif
