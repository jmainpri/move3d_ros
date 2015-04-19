#ifndef CAMERALISTENER_HPP
#define CAMERALISTENER_HPP

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>


namespace HRICS
{
    class CameraListener
    {
        public:
            CameraListener(ros::NodeHandle nh);
            CameraListener(const int id, ros::NodeHandle nh);
            image_transport::Subscriber _sub;
            image_transport::Publisher _pub;
            int _file;
            void imageConverter(const sensor_msgs::ImageConstPtr& msg);
            void setId(int id) {_id = id;}
            int getId() { return _id;}
            cv_bridge::CvImagePtr _current_img;
            void takeSnapshot(timeval time);
            bool pubImage(timeval time);
            ros::NodeHandle nh_;
            bool _is_recording;
            void setFolder(const std::string& foldername) { _folder = foldername; }
            std::string getFolder() { return _folder; }
            int getNumMisses() { return nb_consecutive_misses; }


        private:
            int nb_consecutive_misses;
            int _id;
            std::string _folder;
    };
}

#endif // CAMERALISTENER_HPP

