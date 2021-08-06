#include "opencv2/opencv.hpp"
#include <ros/ros.h>

int main(){
    std::string filepath = "/home/hashimoto/Videos/capture/opencv.mp4";
    cv::VideoCapture video(filepath);
    cv::Mat image_mat;
    cv::VideoWriter out;

    while(1){
        video >> image_mat;
        if (image_mat.empty() == true) break;
        cv::imshow("showing", image_mat);
    }
}