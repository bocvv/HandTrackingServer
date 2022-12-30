#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <map>

#include <opencv2/opencv.hpp>

#include "stabilizer.h"
#include "pose_estimator.h"
#include "scorer.h"

using namespace std;

std::vector<std::string> split(const std::string& s, char delimiter) {
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

map<int, vector<cv::Point3f>> load_landmarks(char* filename) {
    map<int, vector<cv::Point3f>> frame2landmarks;

    ifstream infile(filename);
    string line;
    int frame_id = -1;
    float x, y, z;
    while (getline(infile, line)) {
        if (line.rfind("FRAME_NUM", 0) == 0) {
            frame_id = stoi(split(line, ' ')[1]);
            continue;
        } 
        if (line.rfind("Landmark", 0) == 0) {
            continue;
        }
        if (line.rfind("x:", 0) == 0) {
            x = stof(split(line, ' ')[1]);
            continue;
        }  
        if (line.rfind("y:", 0) == 0) {
            y = stof(split(line, ' ')[1]);
            continue;
        }  
        if (line.rfind("z:", 0) == 0) {
            z = stof(split(line, ' ')[1]);
            if (frame2landmarks.find(frame_id) != frame2landmarks.end()) {
                frame2landmarks[frame_id].push_back(cv::Point3f(x, y, z));
            } else {
                vector<cv::Point3f> landmarks;
                landmarks.push_back(cv::Point3f(x, y, z));
                frame2landmarks[frame_id] = landmarks; 
            } 
            continue;
        }  
    }
    infile.close(); 
    return frame2landmarks;
}

int main(int argc, char** argv) {
    const char* video_src = argv[1];
    cv::VideoCapture cap(video_src);

    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
        
    cv::Mat frame;
    cap >> frame;
    cv::Size img_size(frame.cols, frame.rows);
    //PoseEstimator pose_estimator(cv::Size(frame.cols, frame.rows));
    Scorer scorer(img_size);

    map<int, vector<cv::Point3f>> frame2posemarks = load_landmarks(argv[2]);
    map<int, vector<cv::Point3f>> frame2facemarks = load_landmarks(argv[3]);
    map<int, vector<cv::Point3f>> frame2irismarks = load_landmarks(argv[4]);

    int frame_id = 1;
    while(1) {
        FrameInfo frame_info;
        if (frame2posemarks.find(frame_id) != frame2posemarks.end()) {
            frame_info.pose_landmarks = frame2posemarks[frame_id];
        }
        if (frame2facemarks.find(frame_id) != frame2facemarks.end()) {
            frame_info.face_landmarks = frame2facemarks[frame_id]; 
        }
        if (frame2irismarks.find(frame_id) != frame2irismarks.end()) {
            frame_info.iris_landmarks = frame2irismarks[frame_id];
        }
	    //cout << frame_info.pose_landmarks.size() << endl;
	    //cout << frame_info.face_landmarks.size() << endl;
	    //cout << frame_info.iris_landmarks.size() << endl;
        BehavioralClus clus = scorer.predict(frame_info);  
        cout << clus.to_string() << endl; 
        AttentionState state = scorer.score_attention(clus);
        if (state == AttentionState::MEDIUM) {
            cout << frame_id << " " << "medium" << static_cast<int>(state) << endl;
        }
        if (state == AttentionState::LOW) {
            cout << frame_id << " " << "low" << endl;
        }
        // Capture frame-by-frame
        cap >> frame;
        // If the frame is empty, break immediately
        if (frame.empty())
            break;
        frame_id += 1; 
    }
 
    // When everything done, release the video capture object
    cap.release();
    return 0;
}
