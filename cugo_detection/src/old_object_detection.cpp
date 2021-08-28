#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
ros::Publisher msg_pub;

public:
// コンストラクタ
ImageConverter() : it_(nh_){
        // カラー画像をサブスクライブ
        image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
        // 処理した画像をパブリッシュ
        image_pub_ = it_.advertise("/image_topic", 1);
        msg_pub = nh_.advertise<std_msgs::String>("/msg_topic", 1, true);
}

// デストラクタ
~ImageConverter(){
        cv::destroyWindow(OPENCV_WINDOW);
}
// コールバック関数
void imageCb(const sensor_msgs::ImageConstPtr& msg){
        std_msgs::String str;
        cv::Point2f center, p1;
        float radius;

        cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
        try{
                // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }

        cv::Mat hsv_image, color_mask, gray_image, bin_image, cv_image2, cv_image3;
        // RGB表色系をHSV表色系へ変換して、hsv_imageに格納
        cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

        // 色相(Hue), 彩度(Saturation), 明暗(Value, brightness)
        // 指定した範囲の色でマスク画像color_mask(CV_8U:符号なし8ビット整数)を生成
        // マスク画像は指定した範囲の色に該当する要素は255(8ビットすべて1)、それ以外は0
        //cv::inRange(hsv_image, cv::Scalar(0, 0, 100, 0) , cv::Scalar(180, 45, 255, 0), color_mask);       // 白
        // cv::inRange(hsv_image, cv::Scalar(150, 100, 50, 0), cv::Scalar(180, 255, 255, 0), color_mask);  // 赤
        cv::inRange(hsv_image, cv::Scalar(0, 100, 50, 0), cv::Scalar(30, 255, 255, 0), color_mask);  // 赤
        // cv::inRange(hsv_image, cv::Scalar(20, 50, 50, 0) , cv::Scalar(100, 255, 255, 0), color_mask);   // 黄

        // ビット毎の論理積。マスク画像は指定した範囲以外は0で、指定範囲の要素は255なので、ビット毎の論理積を適用すると、指定した範囲の色に対応する要素はそのままで、他は0になる。
        cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_image2, color_mask);
        // グレースケールに変換
        cv::cvtColor(cv_image2, gray_image, CV_BGR2GRAY);
        // 閾値70で2値画像に変換
        cv::threshold(gray_image, bin_image, 80, 255, CV_THRESH_BINARY);

        // エッジを検出するためにCannyアルゴリズムを適用
        //cv::Canny(gray_image, cv_ptr3->image, 15.0, 30.0, 3);

        // ウインドウに円を描画
        //cv::circle(cv_ptr->image, cv::Point(100, 100), 20, CV_RGB(0,255,0));

        // 輪郭を格納するcontoursにfindContours関数に渡すと輪郭を点の集合として入れてくれる
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  // 輪郭線を格納

        // 各輪郭をcontourArea関数に渡し、最大面積を持つ輪郭を探す
        double max_area = 0;
        int max_area_contour = -1;
        if(contours.size() > 0) {
                for(int j=0; j<contours.size(); j++) {
                        double area = cv::contourArea(contours.at(j));
                        if( max_area < area ) {
                                max_area = area;
                                max_area_contour=j;
                        }
                }
        }
        else{
                ROS_INFO("target nothing!");
                str.data = "stop";
                msg_pub.publish(str);
                return;
        }
        if(max_area_contour == -1) {
                ROS_INFO("target nothing( max_area )!");
                str.data = "stop";
                msg_pub.publish(str);
                return;
        }
        // 最大面積を持つ輪郭の最小外接円を取得
        // std::cout << "error: " << contours.size() << std::endl;
        cv::minEnclosingCircle(contours.at(max_area_contour), center, radius);
        // 最小外接円を描画(画像、円の中心座標、半径、色、線の太さ)
        cv::circle(cv_ptr->image, center, radius, cv::Scalar(255,0,0),3,4);

        // 画面中心から最小外接円の中心へのベクトルを描画
        p1 = cv::Point2f(cv_ptr->image.size().width/2,cv_ptr->image.size().height/2);
        cv::arrowedLine(cv_ptr->image, p1, center, cv::Scalar(0, 255, 0, 0), 3, 8, 0, 0.1);

        // 画像サイズは縦横1/4に変更
        cv::Mat cv_half_image, cv_half_image2, cv_half_image3, cv_half_image4, cv_half_image5;
        cv::resize(cv_ptr->image, cv_half_image,cv::Size(),1.0,1.0);
        cv::resize(gray_image, cv_half_image4,cv::Size(),1.0,1.0);
        if( 0 <= center.x && center.x <= 240) {
                str.data = "turn left";
                msg_pub.publish(str);
                ROS_INFO("turn left");
        }
        else if( 400 < center.x && center.x <= 640) {
                str.data = "turn right";
                msg_pub.publish(str);
                ROS_INFO("turn rigdddddddddht");
        }
        else{
                if(radius >= 130.0) {
                        str.data = "stop";
                        msg_pub.publish(str);
                        ROS_INFO("stop");
                }
                else{
                        str.data = "go ahead";
                }
        }
        // ROS_INFO("{x:%f, y:%f}", center.x, center.y);
        // ROS_INFO("size:%d", cv_ptr->image.size().width);
        ROS_INFO("radius = %f", radius);

        // ウインドウ表示
        cv::imshow("Original Image", cv_half_image);
        cv::imshow("Gray Image", cv_half_image4);
        cv::waitKey(3);
}
};

int main(int argc, char** argv)
{
        ros::init(argc, argv, "object_detection");
        ImageConverter ic;
        ros::spin();
        return 0;
}
