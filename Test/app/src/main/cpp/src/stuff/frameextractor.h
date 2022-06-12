#ifndef ucoslam_FrameExtractor_H
#define ucoslam_FrameExtractor_H
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include "basic_types/frame.h"
#include "imageparams.h"
namespace ucoslam {



/** This class process the input image(s) and creates the Frame that will be used for processing
 * 此类处理输入图像并创建将用于处理的帧
  */
class FrameExtractor{
public:



    FrameExtractor();
     void setParams(  const ucoslam::Params &params);

    void process(const cv::Mat &image, const ImageParams &ip,Frame &frame, uint32_t frameseq_idx=std::numeric_limits<uint32_t>::max(), uint32_t fidx=std::numeric_limits<uint32_t>::max())throw (std::exception);
    void process_rgbd(const cv::Mat &image, const cv::Mat &depthImage,const ImageParams &ip,Frame &frame, uint32_t frameseq_idx=std::numeric_limits<uint32_t>::max(), uint32_t fidx=std::numeric_limits<uint32_t>::max())throw (std::exception);

    //enables/disables removing keypoints into the markers
    bool &removeFromMarkers(){return _removeFromMarkers;}
    cv::Mat getImage()const{return _imgrey;}
    bool &detectMarkers(){return _detectMarkers;}
    bool &detectKeyPoints(){return _detectKeyPoints;}

     void setCounter(uint32_t frame){_counter=frame;}


    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

    //returns the scale factor of the indicated octave

private:

    cv::Mat _imgrey;
    uint32_t _counter=0;//为每个处理过的帧分配一个唯一的id
    bool _removeFromMarkers=true;
    bool _detectMarkers=true;
    bool _detectKeyPoints=true;

    aruco::MarkerDetector _mdetector;
     float _markerSize=0 ;

};

}
#endif
