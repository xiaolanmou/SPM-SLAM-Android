#include <jni.h>
#include <string>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <librealsense2/rs.h>
#include "tic_toc.h"
#include "example/stereo_inertial.h"
#include "log_cout.h"
#include "slam.h"
#include "mapviewer.h"
#include "stuff/timers.h"
#include <aruco/posetracker.h>
#include <Eigen/Geometry>
#include <thread>
#include <unistd.h>


using namespace cv;
using namespace std;
using namespace Example;
bool bContinueThread = false;

string markerConfig = "/sdcard/SPMSLAM-Config/Marker/LiveConfig.yaml";
string cameraInstric = "/sdcard/SPMSLAM-Config/Marker/RealSense_RGB.yaml";
Stereo_Inertial stero_inertial ;
cv::Mat imLeft,imLeftRgb,in_image;
bool bHaveStereoNew;
std::mutex showImgMutex;
std::condition_variable condImgRec;

void process()
{
    while(bContinueThread){
        stero_inertial.runSlam(imLeft);
        std::unique_lock<std::mutex> lk(showImgMutex);
        cv::imwrite("sdcard/left2.png",imLeft);
        cout << "imLeft.size==============>>>>>>>>>>>>>" <<  imLeft.size << endl;
        imLeftRgb = imLeft.clone();
        cout << "最后来这儿了" << endl;
        bHaveStereoNew = true;
        condImgRec.notify_one();
    }


}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_test_MainActivity_nativeOrbInitialised(JNIEnv *env, jobject thiz) {
    //Open Realsense device
    cout << "第三步：初始化" << endl;
    bool flag = stero_inertial.startSensor(markerConfig,cameraInstric,in_image);
    cout << "第四步就是跑算法=====================>>>>>>>>>>>>>>>>>" << endl;
    if(flag)
    {
        bContinueThread = true;
        //开始slam算法
        cout<<"开始SLAM算法*********"<<endl;
        std::thread processThread = std::thread(process);
        processThread.detach();

    }

}
extern "C"
JNIEXPORT void JNICALL
Java_com_example_test_D435iView_nativeCheckCameraNew(JNIEnv *env, jobject thiz,
                                                     jlong left_image_rgb) {
    cout << "第2.1步:判断是否进入" << bHaveStereoNew << endl;
    Mat &imageLeftRgb = * (Mat *)left_image_rgb;
    {
        std::unique_lock<std::mutex> lk(showImgMutex);
        if (!bHaveStereoNew)
        {
            cout << "2.2：等待唤醒" << endl;
            condImgRec.wait(lk);
        }
        cout << "2.3成功唤醒" << endl;
        imageLeftRgb = imLeftRgb.clone();
//        cout << "imageLeftRgb.size()=================>>>>>>>>>>>>>>>>" << imageLeftRgb.size() << endl;
        bHaveStereoNew = false;
    }

}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_test_MainActivity_nativeOrbUninitialised(JNIEnv *env, jobject thiz) {
    cout<<"nativeOrbUninitialised"<<endl;
    stero_inertial.stopSensor();
}


extern "C"
JNIEXPORT void JNICALL
Java_com_example_test_MainActivity_nativeStartLogger(JNIEnv *env, jobject thiz) {
    Log::Log_Cout::start_logger("SPM-SLAM");
    cout.precision(16); // 设置精度16
}


