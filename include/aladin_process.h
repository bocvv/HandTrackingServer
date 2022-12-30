/*************************************************************************
	> File Name: aladin_process.h
	> Author: 
	> Mail: 
	> Created Time: 一  8/31 14:34:42 2020
 ************************************************************************/

#ifndef _ALADIN_PROCESS_H
#define _ALADIN_PROCESS_H

#include "aladin_process.h"
#include "model_manager.h"
#include "scorer.h"
#include "smtp.h"

#define STATISTIC_FREQUENCE 175  // 10 secs
#define CELLPHONE_THRESHOLD 50
#define IRIS_PORT 8765
#define FACE_PORT 8766
#define UPPER_BODY_PORT 8767
#define OBJ_TRACKING_PORT 8768
// #define MULTI_HAND_PORT 8769
#define HAND_PORT 8769

using namespace std;

typedef struct _StudyTimeStatistic {
    float study_hard;
    float offset_head;
    float fall_down;
    float sleepy;
    float eye_close;
    float shoulder_slope;
}StudyTimeStatistic;

class AladinProcessor {
public:
    AladinProcessor() {};
    ~AladinProcessor();

    void init(string bad_pose_path, string use_phone_path);
    int sound_alert(string sound_path);
    void check_bad_poses();
    void process();
    void gen_html();

private:
    // alert sound module
    string pose_path;
    string phone_path;
    // ISoundEngine *sound_engine{nullptr};

    // model manager
    IrisTrackingManager *iris_mgr{nullptr}; 
    FaceManager *face_mgr{nullptr};
    UpperBodyManager *upper_body_mgr{nullptr};
    ObjectTrackingManager *obj_mgr{nullptr};
    HandManager *hand_mgr{nullptr};

    // score strategy
    // info of each frame && predicted result from FrameInfo
    FrameInfo frame_info;
    BehavioralClus behavioral_clus;

    // during one period
    vector<BehavioralClus> behavioral_clus_list;
    AttentionState attention_state;
    HealthState health_state;

    // total study period
    string start_time;
    string end_time;
    float total_min;
    vector<AttentionState> attention_states_vec;
    vector<HealthState> health_states_vec;
    AttentionState total_attention_state;
    HealthState total_health_state;

    // pose sum
    typedef struct _PoseNum {
        int presence_num;
        int look_away_num;
        int yawning_num;
        int close_eye_num;
        int wrong_shoulder_num;
        int wrong_head_num;
        int face_down_num;       
    }PoseNum;
    PoseNum period_pose_num;
    PoseNum total_pose_num;

    int total_frames{0};

    void update_pose_num();

    // check if playing cell phone
    bool has_triggered_phone_check{false};
    int count_phone_checked{0};

    bool is_too_close_to_phone();
    void check_cellphone();

    // smtp email
    CSmtp smtp;
    void send_email(string content);
};


#endif
