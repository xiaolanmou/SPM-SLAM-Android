#ifndef SPM_SLAM_ANDROID_RGB_H
#define SPM_SLAM_ANDROID_RGB_H

#include <condition_variable>
#include <iostream>
#include <thread>
#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"
#include <opencv2/core/core.hpp>
#include "slam.h"
#include "mapviewer.h"
#include "stuff/timers.h"
#include <aruco/posetracker.h>
#include <Eigen/Geometry>
#include <thread>
#include <unistd.h>
using namespace std;
using namespace cv;


namespace Example
{
    class Stereo_Inertial
    {
    public:
        bool startSensor(const string markerconfig,const string cameraconfig,Mat &image);
        void stopSensor();
        void runSlam(Mat &imLeft);
        void process(Mat &img);

        //SLAM
        void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz);
        void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp);
        void LoadCameraPose(const cv::Mat &Tcw);
    private:
        string marker_file,camera_file;
        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline mPipeline;
        rs2::pipeline_profile pipe_profile;
        cv::Mat imCV, imRightCV;
        //条件变量  https://www.jianshu.com/p/c1dfa1d40f53
        std::mutex mutex;
        std::condition_variable cond_image_rec;
        bool image_ready = false;
        double timestamp_image = -1.0;
        int width_img = 640 , height_img = 480;
        //slam
        ucoslam::Slam Slam;
        ucoslam::ImageParams image_params;
        ucoslam::Params params;
//        cv::Mat in_image;  //就是上面的imCV
        ucoslam::TimerAvrg Fps;
        char k=0;
        std::map<int, cv::Mat> frame_pose_map1;  // set of poses and the frames they were detected


    };


}





#endif //SPM_SLAM_ANDROID_RGB_H
