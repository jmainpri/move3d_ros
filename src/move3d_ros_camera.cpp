#include "move3d_ros_camera.hpp"

using namespace std;
using namespace HRICS;

CameraListener::CameraListener(ros::NodeHandle nh) : nh_(nh)
{
    cout << "Enter constructer for camera" << endl;
    setId(0);
    image_transport::ImageTransport it(nh);
    _is_recording = false;
    _file = 0;
    nb_consecutive_misses = 0;

    string home(getenv("HOME"));
    _folder = home + "/workspace/statFiles/snapshots/";

    cout << "start subscriber" << endl;
    _sub = it.subscribe("camera/rgb/image_raw", 1, &CameraListener::imageConverter, this);

    cout << "start publisher" << endl;
    _pub = it.advertise("orkinect/kinect", 1);

}

CameraListener::CameraListener(const int id, ros::NodeHandle nh) : nh_(nh)
{
    cout << "Enter constructer for camera" << endl;
    setId(id);
    _is_recording = false;
    nb_consecutive_misses = 0;
    image_transport::ImageTransport it(nh);
    _file = 0;


    cout << "start subscriber" << endl;
    std::stringstream s;
    s << "camera" << _id << "/rgb/image_raw";
    _sub = it.subscribe(s.str().c_str(), 1, &CameraListener::imageConverter, this);

    cout << "start publisher" << endl;
    s.str( "" );
    s.clear();

    s << "orkinect/kinect" << _id << "/";
    _pub = it.advertise(s.str().c_str(), 1);

}

void CameraListener::imageConverter(const sensor_msgs::ImageConstPtr& msg)
{
    _current_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

}

void CameraListener::takeSnapshot(timeval time)
{
    try
    {
        std::stringstream s;
        s << _folder << _id << "_" << time.tv_sec << "_" << time.tv_usec << ".png";
        //        s << _folder << _id << "_" << _file++ << ".png";
        cv::imwrite(s.str().c_str(), _current_img->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

bool CameraListener::pubImage(timeval time)
{
    std::stringstream file;
    file << _folder << _id << "_" << time.tv_sec << "_" << time.tv_usec << ".png";


    cv::Mat image = cv::imread( file.str() );
    if( image.empty() )
    {
        nb_consecutive_misses++;
        return false;
    }

    nb_consecutive_misses = 0;

    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = image;
    out_msg.header.stamp = ros::Time::now();

    image.release();

    _pub.publish(out_msg.toImageMsg());

    return true;
}


