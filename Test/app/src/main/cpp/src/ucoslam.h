#ifndef UCOSLAM_H
#define UCOSLAM_H
#include <vector>
#include <cstdint>
#include <string>
#include <iostream>
#include <aruco/cameraparameters.h>
#include <aruco/markerdetector.h>
namespace ucoslam {

//types of descriptors that can be used


//SLAM 的处理参数
struct Params{
    Params();
    bool detectMarkers=true;
    bool aprilTag = false;
     float effectiveFocus=-1;//值以启用跨不同相机和分辨率的标准化
    bool removeKeyPointsIntoMarkers=true;
    bool forceInitializationFromMarkers=false;
    float minDescDistance=50;//考虑可能匹配的描述符之间的最小距离
    float baseline_medianDepth_ratio_min=0.01;
    int projDistThr=15;//通过投影搜索点时，搜索半径的最大 2d 距离
     std::string global_optimizer= "g2o";//使用什么全局优化
    int minNumProjPoints=3;//一个点最少被关键帧观测到的数量 minimum number of keyframes in which a point must be seen to keep it
    float keyFrameCullingPercentage=0.8;
    int fps=30;//视频序列的每秒帧数
    float thRefRatio=0.9;//在当前帧中找到的匹配项与参考关键帧相比的比率，以考虑插入新的关键帧
    int maxFeatures=2000;
    int nOctaveLevels=8;
    float scaleFactor=1.2;


    int maxVisibleFramesPerMarker=10;
    float minBaseLine=0.07;//关键帧之间的最小首选距离
    float max_makr_rep_err = 2.5;


    float aruco_markerSize=1;            
    //最小帧数
    int aruco_minNumFramesRequired=3;       
    //考虑初始姿势有效的两个解决方案之间的最小误差率     
    float aruco_minerrratio_valid=3;
    bool aruco_allowOneFrameInitialization=false;
    //当前帧和当前关键帧之间的最小旋转以将其添加为关键帧
    float aruco_maxRotation=0.1;//minimum rotation between current frame and current keyframe to add this as keyframe
    float min_distR = 1.2;
    float max_theta = 32;
    float delta_theta = 1;
    aruco::MarkerDetector::Params aruco_DetectorParams;


    void toStream(std::ostream &str);
    void fromStream(std::istream &str);
    uint64_t getSignature()const;


    void saveToYMLFile(const std::string &path);
    void readFromYMLFile(const std::string &path);
    //--- do not use
    bool runSequential=false;//avoid parallel processing

    private:
    template<typename Type>
    void attemtpRead(const std::string &name,Type &var,cv::FileStorage&fs ){
        if ( fs[name].type()!=cv::FileNode::NONE)
            fs[name]>>var;
//        else
//            std::cout << name << "参数读取失败" << std::endl;
    }


};




}

#endif
