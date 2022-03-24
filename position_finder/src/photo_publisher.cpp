#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class  PhotoPublisher{
    private:
    int counter;
    ros::Publisher pub_;
    public:
    double fps = 0;
    double target_fps = 5;
    PhotoPublisher(ros::NodeHandle *nh):nh_(nh) {
        counter = 0;
        pub_ = nh_->advertise<sensor_msgs::Image>("/photo", 10);    
        cap_ = VideoCapture("/home/parallels/copa5/video/500m.mp4");
        fps = cap_.get(CAP_PROP_FPS);
    }
    void publish_cadr(){
        counter += 1;
        Mat frame;
        Mat frame_gray;
        cap_ >> frame;
        if(counter > fps/target_fps){
            counter = 0;
            std_msgs::Header header; 
            header.stamp = ros::Time::now();
            sensor_msgs::Image img_msg;
            bridge_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
            bridge_.toImageMsg(img_msg);
            pub_.publish(img_msg);
        }
    }
    protected:
    ros::NodeHandle* nh_;
    VideoCapture cap_;
    cv_bridge::CvImage bridge_;
    
};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    PhotoPublisher pb = PhotoPublisher(&nh);
    ros::Timer pb_video =
        nh.createTimer(ros::Duration(1/pb.fps),
                       std::bind(&PhotoPublisher::publish_cadr, pb));
    ros::spin();
    // ros::Rate loop_rate(static_cast<int>(pb.fps));
    // while(ros::ok()){
    //     pb.publish_cadr();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}