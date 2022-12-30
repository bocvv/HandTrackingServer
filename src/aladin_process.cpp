/*************************************************************************
	> File Name: aladin_process.cpp
	> Author: Bowei Wang
	> Mail: wangbw@rd.neteasy.com
	> Created Time: 一  8/31 14:33:57 2020
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <thread>
#include <sys/time.h>
#include <unistd.h>
#include <csignal>
#include <opencv.hpp>
#include <core/base.hpp>
#include "aladin_process.h"

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

using namespace std;

#define BAD_POSE_THRESHOLD 100

bool has_triggered = false;
int bad_poses = 0;

#ifdef SAVE_RAW_VIDEO
cv::Mat g_front_frame;
cv::Mat g_top_frame;

int read_from_front_mjpg_streamer() {
    cv::VideoCapture cap;
    cap.open("http://10.211.55.4:8123/?action=stream?dummy=param.mjpg");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 800);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 600);
    cap.set(cv::CAP_PROP_FPS, 20);

    while (1) {
        cap >> g_front_frame;
    }
}

int read_from_top_mjpg_streamer() {
    cv::VideoCapture cap;
    cap.open("http://10.211.55.4:8124/?action=stream?dummy=param.mjpg");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 20);

    while (1) {
        cap >> g_top_frame;
    }
}
#endif
 

namespace {
    volatile std::sig_atomic_t m_stop;

    string get_timestamp() {
        struct timeval now_time;
        gettimeofday(&now_time, NULL);
        time_t tt = now_time.tv_sec;
        tm *temp = localtime(&tt);
        char time_char[32]={'\0'};
        sprintf(time_char, "%04d-%02d-%02d %02d:%02d:%02d", temp->tm_year+ 1900, temp->tm_mon+1, temp->tm_mday,temp->tm_hour, temp->tm_min, temp->tm_sec);
        string time_str = time_char;
        return time_str;
    }

    static double tick_tok(void)
    {
        struct timeval t;
        gettimeofday(&t, 0);
        return t.tv_sec + 1E-6 * t.tv_usec;
    }

    static void app_msg_pump(int sig) {
        if (sig == SIGINT || sig == SIGTERM) {
            m_stop = sig;
        }
    }

    static float distance_between(cv::Point3f pt1, cv::Point3f pt2, int img_w, int img_h) {
        float delta_x = pt1.x;
        float delta_y = pt1.y;
        delta_x = (delta_x - pt2.x) * img_w;
        delta_y = (delta_y - pt2.y) * img_h;
        return std::sqrt(delta_x * delta_x + delta_y * delta_y);
    }

    static void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
        ma_decoder* pDecoder = (ma_decoder*)pDevice->pUserData;
        if (pDecoder == NULL) {
            return;
        }

        ma_decoder_read_pcm_frames(pDecoder, pOutput, frameCount);

        (void)pInput;
    }
}

AladinProcessor::~AladinProcessor() {
    if (!iris_mgr) {
        delete iris_mgr;
    }
    if (!face_mgr) {
        delete face_mgr;
    }
    if (!upper_body_mgr) {
        delete upper_body_mgr;
    }
#ifdef WITH_PHONE_CHECKING
    if (!obj_mgr) {
        delete obj_mgr;
    }
    if (!hand_mgr) {
        delete hand_mgr;
    }
#endif
}

void AladinProcessor::init(string bad_pose_path, string use_phone_path) {
    pose_path = bad_pose_path;
    phone_path = use_phone_path;
    iris_mgr = new IrisTrackingManager();
    face_mgr = new FaceManager();
    upper_body_mgr = new UpperBodyManager();
#ifdef WITH_PHONE_CHECKING
    obj_mgr  = new ObjectTrackingManager();
    hand_mgr = new HandManager();
#endif

    // init number
    period_pose_num = {.presence_num = 0,
                      .look_away_num = 0,
                      .yawning_num = 0,
                      .close_eye_num = 0,
                      .wrong_shoulder_num = 0,
                      .wrong_head_num = 0,
                      .face_down_num = 0};
    total_pose_num = {.presence_num = 0,
                      .look_away_num = 0,
                      .yawning_num = 0,
                      .close_eye_num = 0,
                      .wrong_shoulder_num = 0,
                      .wrong_head_num = 0,
                      .face_down_num = 0};

    // init smtp params
    smtp = CSmtp(
		25,									/*smtp端口*/
		"smtp.qq.com",						/*smtp服务器地址*/
		"732887477@qq.com",				        /*你的邮箱地址*/
		"znsayylgmoybbddi",					        /*邮箱密码*/
		"aladin_test@126.com",					    /*目的邮箱地址*/
		"test",							/*主题*/
		"test for aladin"		                /*邮件正文*/
	);
}

int AladinProcessor::sound_alert(string sound_path) {
    if (sound_path.empty()) {
        cout << "No input file." << endl;
        return -1;
    }
    ma_result result;
    ma_decoder decoder;
    ma_device_config deviceConfig;
    ma_device device;

    result = ma_decoder_init_file(sound_path.c_str(), NULL, &decoder);
    if (result != MA_SUCCESS) {
        return -2;
    }

    deviceConfig = ma_device_config_init(ma_device_type_playback);
    deviceConfig.playback.format   = decoder.outputFormat;
    deviceConfig.playback.channels = decoder.outputChannels;
    deviceConfig.sampleRate        = decoder.outputSampleRate;
    deviceConfig.dataCallback      = data_callback;
    deviceConfig.pUserData         = &decoder;

    if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
        cout << "Failed to open playback device." << endl;
        ma_decoder_uninit(&decoder);
        return -3;
    }

    if (ma_device_start(&device) != MA_SUCCESS) {
        cout << "Failed to start playback device." << endl;
        ma_device_uninit(&device);
        ma_decoder_uninit(&decoder);
        return -4;
    }

    sleep(3);

    ma_device_uninit(&device);
    ma_decoder_uninit(&decoder);
}

void AladinProcessor::check_bad_poses() {
    sleep(5);
    if (bad_poses > BAD_POSE_THRESHOLD) {
        cout << bad_poses << endl;
        int r = sound_alert(pose_path);
        bad_poses = 0;
    }
    has_triggered = false;
}

void AladinProcessor::update_pose_num() {
    if (behavioral_clus.presence) {
        total_pose_num.presence_num++;
    }
    if (behavioral_clus.look_away) {
        total_pose_num.look_away_num++;
    }
    if (behavioral_clus.yawning) {
        total_pose_num.yawning_num++;
    }
    if (behavioral_clus.close_eye) {
        total_pose_num.close_eye_num++;
    }
    if (behavioral_clus.wrong_shoulder) {
        total_pose_num.wrong_shoulder_num++;
    }
    if (behavioral_clus.wrong_head) {
        total_pose_num.wrong_head_num++;
    }
    if (behavioral_clus.face_down) {
        total_pose_num.face_down_num++;
    }
}

void AladinProcessor::gen_html() {
    ofstream outfile;
    outfile.open("../data/index.html", ios::out | ios::trunc);

    // write to outfile
    outfile << "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />" << "\n";
    outfile << "<style>" << "\n";
    outfile << ".flip-x {" << "\n";
    outfile << "	filter: FlipH; /* IE only */" << "\n";
    outfile << "	-moz-transform: matrix(-1, 0, 0, 1, 0, 0);" << "\n";
    outfile << "	-webkit-transform: matrix(-1, 0, 0, 1, 0, 0);" << "\n";
    outfile << "    -transform: rotate(270deg)" << "\n";
    outfile << "    -ms-transform: rotate(270deg); /* IE 9 */" << "\n";
    outfile << "    -moz-transform: rotate(270deg); /* Firefox */" << "\n";
    outfile << "    -webkit-transform: rotate(270deg); /* Safari and Chrome */" << "\n";
    outfile << "    -o-transform: rotate(270deg); /* Opera */" << "\n";
    outfile << "}" << "\n";
    outfile << "</style>" << "\n";
    outfile << "<h1> 学习日报 <br /> </h1>" << "\n";
    outfile << "<h2> 学习画面 <br /> <br />  <br /> <br /> <br /> </h2>" << "\n";
    outfile << "<img src=\"http://10.211.55.4:8123/?action=stream?dummy=param.mjpg\" class=\"flip-x\" />" << "\n";
    outfile << "<p> <br /> <br /> <br /> <br /> <br /> <br /> </p>" << "\n";
    outfile << "<h2> 学习进程 </h2>" << "\n";
    outfile << "<p> 本次学习时间: " << start_time << " 至 " << end_time << "<br />" << \
        "最长专注时间：" << "  " << "<br />" << "已累计学习: " << "15天！" << "</p>" << "\n" << "\n";
    outfile << "<p> 注意力动态图: </p>" << "\n";
    // graph table
    outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
    outfile << "      <tr>" << "\n";
    float unit_rate = 1.f / attention_states_vec.size();
    for (int i = 0; i < attention_states_vec.size(); i++) {
        string attention_color;
        if (attention_states_vec[i] == AttentionState::HIGH) {
            attention_color = "#4FC3FE";
        } else if (attention_states_vec[i] == AttentionState::MEDIUM) {
            attention_color = "#9BDDFE";
        } else {
            attention_color = "#FFA500";
        }
        outfile << "        <th bgcolor=\"" << attention_color << "\" height=\"100%\" width=\"" << unit_rate << "\"></th>" << "\n";
    }
    outfile << "      <tr>" << "\n";
    outfile << "</table>" << "\n";
    outfile << "<p>                                      深蓝色：健康、专注 </p>" << "\n";
    outfile << "<p>                                      浅蓝色：需改善 </p>" << "\n";
    outfile << "<p>                                      黄色：未在学习 </p>" << "\n";
    // total study comment
    outfile << "<h2> 学习评价 </h2>" << "\n";
    outfile << "<p> 注意力：2星 <br />    孩子的注意力比较集中，较少出现左顾右盼等情况，全程也没有困倦.</p>" << "\n";
    outfile << "<p> 健康度：1星 <br />    孩子坐姿不太健康，多次出现偏头、趴着等情况。请提醒孩子注意坐姿，以免对视力和健康造成影响.</p>" << "\n";
    outfile << "<p> 学习时间：1星 <br />    学习时间过于零散，单次时长在30分～45分钟为宜.</p>" << "\n";
    // study pose sum
    outfile << "<h2> 学习状态 </h2>" << "\n";
    cout << "total_frames" << total_frames << endl;
    if (total_pose_num.presence_num > 0) {
        float rate = (float)total_pose_num.presence_num / total_frames;
        outfile << "<p> 认真学习(" << total_min * rate <<" 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }
    if (total_pose_num.look_away_num > 0) {
        float rate = (float)total_pose_num.look_away_num / total_frames;
        cout << "look_away_num " << total_pose_num.look_away_num << endl;
        cout << "rate" << rate << endl;
        outfile << "<p> 转移目光(" << total_min * rate << " 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }
    if (total_pose_num.yawning_num > 0) {
        float rate = (float)total_pose_num.yawning_num / total_frames;
        outfile << "<p> 困倦(" << total_min * rate << " 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }
    if (total_pose_num.close_eye_num > 0) {
        float rate = (float)total_pose_num.close_eye_num / total_frames;
        outfile << "<p> 闭眼(" << total_min * rate << " 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }
    if (total_pose_num.wrong_shoulder_num > 0) {
        float rate = (float)total_pose_num.wrong_shoulder_num / total_frames;
        outfile << "<p> 肩斜(" << total_min * rate << " 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }
    if (total_pose_num.wrong_head_num > 0) {
        float rate = (float)total_pose_num.wrong_head_num / total_frames;
        outfile << "<p> 偏头(" << total_min * rate  << " 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }
    if (total_pose_num.face_down_num > 0) {
        float rate = (float)total_pose_num.face_down_num / total_frames;
        outfile << "<p> 趴桌(" << total_min * rate << " 分钟) <br /> </p>" << "\n";
        outfile << "<table height=\"20\" width=\"1000\" frame=\"void\" rules=\"none\">" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "        <th bgcolor=\"#4FC3FE\" height=\"100%\" width=\"" << rate << "\"></th>" << "\n";
        outfile << "        <th bgcolor=\"#D3D3D3\" height=\"100%\" width=\"" << 1 - rate << "\"></th>" << "\n";
        outfile << "      <tr>" << "\n";
        outfile << "</table>" << "\n";
    }

    outfile.close();
}

#ifdef WITH_PHONE_CHECKING
bool AladinProcessor::is_too_close_to_phone() {
    for (auto p : hand_mgr->hand_center_pts) {
        if (distance_between(p, obj_mgr->phone_center_pts, hand_mgr->img_width, \
                             hand_mgr->img_width) < PHONE_DISTANCE) {
            return true;
        }
    }

    return false;
}
#endif

void AladinProcessor::check_cellphone() {
    sleep(8);
    cout << count_phone_checked << endl;
    if (count_phone_checked > CELLPHONE_THRESHOLD) {
        int r = sound_alert(phone_path);
    }
    count_phone_checked = 0;
    has_triggered_phone_check = false;
}

void AladinProcessor::send_email(string content) {
    int err;
    if (content.empty()) {
        content = "NULL MESSAGE";
    }
    smtp.content = content;
	if ((err = smtp.SendEmail_Ex()) != 0)
	{
		if (err == 1)
			cout << "错误1: 由于网络不畅通，发送失败!" << endl;
		if (err == 2)
			cout << "错误2: 用户名错误,请核对!" << endl;
		if (err == 3)
			cout << "错误3: 用户密码错误，请核对!" << endl;
		if (err == 4)
			cout << "错误4: 请检查附件目录是否正确，以及文件是否存在!" << endl;
	}
}

void AladinProcessor::process() {
    // send end-msg at the beginning of study
    send_email("The child begins studying!");

    start_time = get_timestamp();
    double start_tick = tick_tok();

    // signal used to exit
    m_stop = 0;
    std::signal(SIGINT, &app_msg_pump);  // catch signal interrupt request
    std::signal(SIGTERM, &app_msg_pump); // catch signal terminate request

    // initialize
    iris_mgr->setup(IRIS_PORT);
    face_mgr->setup(FACE_PORT);
    upper_body_mgr->setup(UPPER_BODY_PORT);
#ifdef WITH_PHONE_CHECKING
    obj_mgr->setup(OBJ_TRACKING_PORT);
    hand_mgr->setup(HAND_PORT);
#endif

    int period_frame_sum{0};
    frame_info.frame_id = 0;
    Scorer scorer(cv::Size(600, 800));
#ifdef SAVE_RAW_VIDEO
    cv::VideoWriter front_writer;
    cv::VideoWriter top_writer;
    string front_path = "../data/raw_front_video.mp4";
    string top_path = "../data/raw_top_video.mp4";
    front_writer.open(front_path, front_writer.fourcc('a', 'v', 'c', '1'), 25.0, cv::Size(600, 800), true);
    top_writer.open(top_path, top_writer.fourcc('a', 'v', 'c', '1'), 25.0, cv::Size(640, 480), true);

    // open mjpg stream
    std::thread t_read_front(read_from_front_mjpg_streamer);
    t_read_front.detach();
    std::thread t_read_top(read_from_top_mjpg_streamer);
    t_read_top.detach();
    sleep(2);
#endif

    // start processing
    while (m_stop == 0) {
        // get the udp protobuf message and process
        iris_mgr->update();
        face_mgr->update();
        upper_body_mgr->update();
#ifdef WITH_PHONE_CHECKING
        obj_mgr->update();
        hand_mgr->update();
#endif

        // copy landmark data from udp msg to frame_info vector
        frame_info.frame_id += 1; 
        frame_info.iris_landmarks.clear();
        frame_info.face_landmarks.clear();
        frame_info.pose_landmarks.clear();
        frame_info.iris_landmarks.assign(iris_mgr->eye_left_pts.begin(), \
            iris_mgr->eye_left_pts.end());
        frame_info.iris_landmarks.insert(frame_info.iris_landmarks.end(), \
            iris_mgr->eye_right_pts.begin(), iris_mgr->eye_right_pts.end());
        frame_info.face_landmarks.assign(face_mgr->face_pts_group[0].begin(), \
            face_mgr->face_pts_group[0].end());
        frame_info.pose_landmarks.assign(upper_body_mgr->upper_body_pts.begin(), \
            upper_body_mgr->upper_body_pts.end());

        // predict BahaviorralClus of each frame
        behavioral_clus = scorer.predict(frame_info);
        update_pose_num();
        behavioral_clus_list.push_back(behavioral_clus);
        ++period_frame_sum;

        // calculate the score of a period
        if (period_frame_sum == STATISTIC_FREQUENCE) {
            period_frame_sum = 0;;
            attention_state = scorer.score_attention(behavioral_clus_list);
            health_state = scorer.score_health(behavioral_clus_list);
            attention_states_vec.push_back(attention_state);
            health_states_vec.push_back(health_state);
            cout << "attention 10s = " << static_cast<int>(attention_state) << endl;
            cout << "health 10s = " << static_cast<int>(health_state) << endl;

            // alert the bad pose 
            if (health_state == HealthState::LOW) {
               int r = sound_alert(pose_path); 
            }

            // clear the period behavioral_clus
            behavioral_clus_list.clear();
        }

#ifdef WITH_PHONE_CHECKING
        // checking if playing cell phone
        if (obj_mgr->with_phone && is_too_close_to_phone()) {
            ++count_phone_checked;
            if (!has_triggered_phone_check) {
                has_triggered_phone_check = true;  // do not trigger in next 5 seconds
                std::thread t_check_cellphone(&AladinProcessor::check_cellphone, this);
                t_check_cellphone.detach();
            }
        }
#endif

#ifdef SAVE_RAW_VIDEO
        // save raw VideoCapture
        if (g_front_frame.empty()) break;  // End of video.
        cv::Mat camera_frame(800, 600, g_front_frame.depth());
        cv::transpose(g_front_frame, camera_frame);
        cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ -1);
        front_writer << camera_frame;

        if (g_top_frame.empty()) break;  //End of top video
        top_writer << g_top_frame;
#endif
    }

    // score the whole stduy state
    total_frames = frame_info.frame_id;
    total_attention_state = scorer.count_attention(attention_states_vec);
    total_health_state = scorer.count_health(health_states_vec);
    cout << "total attention state = " << static_cast<int>(total_attention_state) << endl;
    cout << "total health state = " << static_cast<int>(total_health_state) << endl;

    // generate the html page
    end_time = get_timestamp();
    total_min = (tick_tok() - start_tick) / 60;
    gen_html();

#ifdef SAVE_RAW_VIDEO
    // writer release
    front_writer.release();
    top_writer.release();
#endif

    // send end-msg after finishing study
    send_email("The child has finished the study!");
}
