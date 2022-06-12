#ifndef ucoslam_Frame_H_
#define ucoslam_Frame_H_
#include <cstdint>
#include <limits>
#include <vector>
#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <aruco/aruco.h>
#include "stuff/picoflann.h"
#include "stuff/se3transform.h"
#include "imageparams.h"
#include "stuff/reusablecontainer.h"
#include "ucoslam.h"
using namespace  std;


namespace ucoslam {


//为给定标记定义 IPPE 算法返回的一组姿势
struct MarkerPosesIPPE
{
    cv::Mat sols[2];
    double errs[2];
    double err_ratio=0;
    double theta=0;
    double distR=0;
    double tau=0;
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

    MarkerPosesIPPE(){}
    MarkerPosesIPPE(const MarkerPosesIPPE&M)
    {
        M.copyTo(*this);
    }
    MarkerPosesIPPE & operator=(const MarkerPosesIPPE&M){
        M.copyTo(*this);
        return *this;
    }

    void copyTo(MarkerPosesIPPE &mposes)const{
        for(int i=0;i<2;i++){
            sols[i].copyTo(mposes.sols[i]);
            mposes.errs[i]=errs[i];
        }
        mposes.err_ratio=err_ratio;
        mposes.distR = distR;
        mposes.theta = theta;
        mposes.tau = tau;
    }

};

//A image frame
class Frame{
    struct KdTreeKeyPoints{
        inline float operator()(const cv::KeyPoint&kp,int dim)const{
            return dim==0?kp.pt.x:kp.pt.y;
        }
    };
public:
//    ~Frame();
    uint32_t idx=std::numeric_limits<uint32_t>::max();//帧标识符
    std::vector<aruco::Marker> markers;//在图像中检测到的一组原始标记
    std::vector<aruco::Marker> und_markers;//检测到去除失真的一组标记
    std::vector< MarkerPosesIPPE> markers_solutions;//IPPE算法估计的解
    picoflann::KdTreeIndex<2,KdTreeKeyPoints> keypoint_kdtree;
     cv::Mat desc;//描述子集合
    std::vector<uint32_t> ids;//对于每个关键点，它所属的 MapPoint（std::numeric_limits<uint32_t>::max() 如果未分配则使用）
    std::vector<char> nonMaxima;
    Se3Transform pose_f2g;//帧位姿约定：全局 -> 此帧 frame pose  Convenion: Global -> This Frame
    std::vector<cv::KeyPoint> und_kpts;//去除失真的一组关键点
    std::vector<cv::KeyPoint> kpts;//原始关键点
    std::vector<float> depth;//depth if rgbd camaera
    cv::Mat image;//grey image(it may not be stored)
    cv::Mat smallImage;//reduced image version employed for ferns database
     std::vector<int> fernCode;
    //帧在其被捕获的序列中的标识符。
    uint32_t fseq_idx=std::numeric_limits<uint32_t>::max();
    //采用的关键点检测器的比例因子
    vector<float> scaleFactors;
    ImageParams imageParams;//camera with which it was taken
    bool isBad=false;
     //从当前信息返回一个地图点观测


    void clear();
    //如果在 Frame 中，则返回带有 id 的标记的位置，否则返回 -1
    //returns the position of the marker with id indicated if is in the Frame, and -1 otherwise
    int getMarkerIndex(uint32_t id)const  ;
    //returns the marker indicated 返回指示的标记
    aruco::Marker getMarker(uint32_t id)const  ;
    //返回所指示标记的 MarkerPosesIPPE 信息
    //returns the MarkerPosesIPPE info on the marker indicated
     MarkerPosesIPPE getMarkerPoseIPPE(uint32_t id)const  ;

    // I/O
    friend ostream& operator<<(ostream& os, const Frame & f)
    {
       os << "Info about the frame:" << f.idx << endl;
       os << "+ Keypoints: " << f.und_kpts.size() << endl;
        return os;
    }

    //给定位姿，返回世界参考系统中的相机中心位置
    cv::Point3f getCameraCenter()const;
    //返回具有相机观察方向的归一化向量
    cv::Point3f getCameraDirection()const;

    std::vector<uint32_t> getKeyPointsInRegion(cv::Point2f p, float radius ,  int minScaleLevel=0,int maxScaleLevel=std::numeric_limits<int>::max()) const;


    //computes a number unique with the current configuration
    uint64_t getSignature()const;

    bool isValid()const{return ids.size()!=0 || und_markers.size()!=0;}
    //---------------------
    //serialization routines
    void toStream(std::ostream &str) const ;
    void fromStream(std::istream &str) ;

    //for internal usage only
    void create_kdtree(){
        keypoint_kdtree.build(und_kpts);
        assert(imageParams.CamSize.area()!=0);
    }

    //returns a pointer to the extra space
    template<typename T> inline T* getExtra(){return (T*)(&extra_1[0]);}
    template<typename T> inline const T* getExtra()const{return (T*)(&extra_1[0]);}

    template<typename T> inline T* getExtra_Mapper(){return (T*)(&extra_1[64]);}
    template<typename T> inline const T* getExtra_Mapper()const{return (T*)(&extra_1[64]);}



    //给定全局坐标中的 3d 点，如果该点不在相机前面或不投影到相机中，则将其投影到帧中返回一个 nan,nan 点
    inline cv::Point2f project(cv::Point3f p3d,bool setNanIfDepthNegative=true,bool setNanIfDoNotProjectInImage=false)const{
            cv::Point3f res;
            const float *rt=pose_f2g.ptr<float>(0);
            res.z=p3d.x*rt[8]+p3d.y*rt[9]+p3d.z*rt[10]+rt[11];
            if (res.z<0 && setNanIfDepthNegative )
                return cv::Point2f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
            res.x=p3d.x*rt[0]+p3d.y*rt[1]+p3d.z*rt[2]+rt[3];
            res.y=p3d.x*rt[4]+p3d.y*rt[5]+p3d.z*rt[6]+rt[7];
            //now, project
            const float *cam=imageParams.CameraMatrix.ptr<float>(0);
            cv::Point2f r2d;
            r2d.x= (cam[0]*res.x/res.z)+cam[2];
            r2d.y= (cam[4]*res.y/res.z)+cam[5];
            if ( setNanIfDoNotProjectInImage){
                if (!( r2d.x>=0 && r2d.y>=0 && r2d.x<imageParams.CamSize.width &&  r2d.y<imageParams.CamSize.height) )
                    return cv::Point2f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
            }
            return r2d;
    }
    inline cv::Point3f get3dStereoPoint(uint32_t kptIdx)const{
        assert(depth[kptIdx]>0);
        cv::Point3f p;
        p.z=depth[kptIdx];
        p.x= ((und_kpts[kptIdx].pt.x-imageParams.cx())*p.z)/ imageParams.fx();
        p.y= ((und_kpts[kptIdx].pt.y-imageParams.cy())*p.z)/ imageParams.fy();
        return p;
    }
    //returns the scale factor  返回尺度因素
    inline float getScaleFactor()const{return scaleFactors.size()==0?1:scaleFactors[1];}

    //returns the list of mappoints ids visible from this frame 返回从此帧可见的地图点 ID 列表
    vector<uint32_t> getMapPoints()const;

    //在向量中设置无效在邻域中没有显着响应的元素
    void nonMaximaSuppresion();

    vector<uint32_t> getIdOfPointsInRegion(cv::Point2f p, float radius);


    //deep copy of the frame
    void copyTo( Frame &f) const;
     Frame & operator=(const Frame &f);
private:
    //extra space for usage
    uint32_t extra_1[128];

};







//! \class FrameSet
//! \brief A set of image frames
class FrameSet :public  ReusableContainer<ucoslam::Frame> {// std::map<uint32_t,ucoslam::Frame>{
public:
    FrameSet(){ }



    //返回要插入的下一帧的 id
    uint32_t getNextFrameIndex()const{return getNextPosition();}

    //! 添加要设置的新帧并返回其 idx（如果未设置，则分配一个新帧）
    inline uint32_t addFrame(const ucoslam::Frame & frame){
         auto  inserted=ReusableContainer<ucoslam::Frame>::insert(frame);
         inserted.first->idx=inserted.second;//set the idx to the inserted frame
         return inserted.first->idx;
    }



    // I/O
    friend ostream& operator<<(ostream& os, const FrameSet & fs)
    {
       os << "Info about the frame-set:" << fs.size() << endl;
       for (const auto &f : fs)  os << f ;

       return os;
    }

    void toStream(ostream &str)const;
    void fromStream(istream &str) ;
    uint64_t getSignature()const;

};

}
#endif
