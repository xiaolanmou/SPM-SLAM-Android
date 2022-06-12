#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <mystring.h>
#include "example/stereo_inertial.h"
#include "../timer.h"


#include "aruco/markerdetector.h"

extern bool bContinueThread;
namespace Example
{
    static int index = 0;
    void Stereo_Inertial::runSlam(Mat &imLeft) {
        index++;
//        cout << "runSLAM中查看imCV图像宽："<<endl;
        {

            std::unique_lock<std::mutex> lk(mutex);
            while(!image_ready)
                cond_image_rec.wait(lk);


            int currentFrameIndex = index;
            cout << "index:" << index << endl;

            cout << "真的是你吗？？？？？" << endl;
            cout << "果然是你" << endl;
//            ucoslam::Frame frameA;
//            ucoslam::Frame frameB;
//            frameA.markers.push_back(aruco::Marker());
//            swap(frameA,frameB);
//            frameA.clear();//都是空的
//            frameB.clear();
//            cout << "这就不能打印的" << endl;
//            aruco::Marker mm ;
//            vector<aruco::Marker> vv;
//            vv.push_back(mm);
//            frame.markers = vv;
//            frame.clear();

//            aruco::MarkerDetector::MarkerCandidate mmeeee;
//            mmeeee.clear();
            cout << "不是你" << endl;
            cout << "haha" << endl;

//            Fps.start();
//            cv::Mat pose = Slam.process(imCV, image_params,currentFrameIndex);//g --> c
//            Fps.stop();
//            //inf一般是因为得到的数值，超出浮点数的表示范围
//            cout << "fps=" << to_string(1./Fps.getAvrg()) << endl;
//            if (!pose.empty())
//                frame_pose_map1.insert({currentFrameIndex,pose});
//                for(auto &m:Slam.TheMap->map_markers){
//                  if (m.second.pose_g2m.isValid())
//                        cout << "id=" << m.second.id << "\t,pose:" << m.second.pose_g2m.convert().inv() << endl;
//            }
//
//            cout<<"Image = "<<Slam.num_of_image<<", tracking = "<<Slam.num_of_success;
//            double rate = Slam.num_of_success/float(Slam.num_of_image);
//            cout<<", Tracking rate = "<<rate<<endl;
//            cerr<<"The markermap is saved to markermap.yml"<<endl;
//            cout << "runSLAM中查看imCV图像宽：" << imCV.cols << endl;
            image_ready = false;
            imLeft = imCV.clone();
        }


    }
    bool Stereo_Inertial::startSensor(const string markerconfig,const string cameraconfig,Mat &image)
    {
        cout<<"3.1打开传感器：Stereo_Inertial startSensor"<<endl;
//        voc_file = strVocFile;
//        setting_file = strSettingsFile;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        rs2::device selected_device;
        if (devices.size() == 0)
        {
            cout<<"Device no connected, please connect a RealSense device======******"<<endl;
            return false;
        }
        else
        {
            selected_device = devices.front();
            cout<<"Device connected===============******"<<endl;
        }
        //关闭结构光
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();
        if(depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED,0);//disable emitter
        }
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR,width_img,height_img,RS2_FORMAT_BGR8,30);
        auto frameHandle = [&](const rs2::frame& frame){
            //realsense要求，锁住
            std::unique_lock<std::mutex> lk(mutex);
//            cout << "3.1.1：查看数据" << endl;
            if(rs2::frameset fs = frame.as<rs2::frameset>()){
                double new_timestamp_image = fs.get_timestamp()*1e-3;
                if(abs(timestamp_image-new_timestamp_image)<0.001){
                    cout<<"Two frames with the same timeStamp!!!"<<endl;
                    return;
                }
//                cout << "说明有数据" << endl;
                rs2::video_frame color_frame = fs.get_color_frame();
//                cout << "color_frame.get_height():" << color_frame.get_height() << endl;
                cv::Mat imColor =  cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()));
                imCV = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()));
                image = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()));
//                cout << " 3.1.1 imCV.size >>>>>>>>>>>>>>>>>====" << imCV.size << endl;
                timestamp_image = fs.get_timestamp()*1e-3;
                image_ready = true;
//                process(imColor);
//                cv::imwrite("/sdcard/color.png",imColor);
            }
//            else{
//                cout << "似乎没有数据"<< endl;
//            }
            lk.unlock();
            cond_image_rec.notify_all();
        };
        pipe_profile = mPipeline.start(cfg,frameHandle);
//        pipe_profile = mPipeline.start(cfg);
        {
            /*
            之前将这段文字放在了runslam中，会导致
            A/libc: Fatal signal 6 (SIGABRT), code -1 (SI_QUEUE) 和一个code 1的错
             一开始以为是yaml文件的读取错误，改了三个地方：

             */

            marker_file = markerconfig;
            camera_file = cameraconfig;
            image_params.readFromXMLFile(cameraconfig);
            cout<<"distorsion = \n"<<image_params.Distorsion<<endl;
            cout<<"AprilTag = "<<image_params.aprilTag<<endl;
            cout<<"in_image size: "<< imCV.size() << endl;
            cout<<"image_params CamSize:"<< image_params.CamSize << endl;
            params.readFromYMLFile(markerconfig);
            auto TheMap=std::make_shared<ucoslam::Map>();
            //Create the viewer to see the images and the 3D
//            auto TViewer = ucoslam::MapViewer::create(  "Cv");
//            TViewer->set("showNumbers","1");
//            TViewer->set("canLeave","1");
//            TViewer->set("mode","0");
//            TViewer->set("modelMatrix","0.998437 -0.0490304 0.0268194 0  0.00535287 0.561584 0.827403 0  -0.0556289 -0.825967 0.560969 0  0 0 0 1");
//            TViewer->set("viewMatrix"," 1 0 0 0.01  0 4.63287e-05 -1 0.910185  0 1 4.63287e-05 9.18  0 0 0 1 ");

            Slam.setParams(TheMap, params );
            cout<<"===@params=== \n";
            cout<<"Marker size "<<params.aruco_markerSize<<endl;
            cout<<"min distance "<<params.minBaseLine<<endl;
            cout<<"min err Ratio "<<params.aruco_minerrratio_valid<<endl;
            cout<<"fx="<<image_params.fx()<<", fy="<<image_params.fy()<<endl;
            cout<<"cx="<<image_params.cx()<<", cy="<<image_params.cy()<<endl;



            cout << "看看SDK自带参数======================>>>>>>>>>" << endl;
            rs2::stream_profile cam = pipe_profile.get_stream(RS2_STREAM_COLOR);
            rs2_intrinsics intrinsics = cam.as<rs2::video_stream_profile>().get_intrinsics();
            width_img = intrinsics.width;
            height_img = intrinsics.height;
            cout << "RGB camera: \n";
            cout << " fx = " << intrinsics.fx << std::endl;
            cout << " fy = " << intrinsics.fy << std::endl;
            cout << " cx = " << intrinsics.ppx << std::endl;
            cout << " cy = " << intrinsics.ppy << std::endl;
            cout << " height = " << intrinsics.height << std::endl;
            cout << " width = " << intrinsics.width << std::endl;
            cout << " Coeff = " << intrinsics.coeffs[0] << ", " << intrinsics.coeffs[1] << ", " <<
                 intrinsics.coeffs[2] << ", " << intrinsics.coeffs[3] << ", " << intrinsics.coeffs[4] << ", " << std::endl;
            cout << " Model = " << intrinsics.model << std::endl;
        }
        bContinueThread = true;

        return true;
    }
    void Stereo_Inertial::process(Mat &img){
    }

    void Stereo_Inertial::stopSensor()
    {
        cout<<"Stereo_Inertial stopSensor"<<endl;
//        mPipeline.stop();
        bContinueThread = false;

    }
    void Stereo_Inertial::getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
        //get the 3d part of matrix and get quaternion
        assert(M_in.total()==16);
        cv::Mat M;
        M_in.convertTo(M,CV_64F);
        cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
        //use now eigen
        Eigen::Matrix3f e_r33;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                e_r33(i,j)=M.at<double>(i,j);

        //now, move to a angle axis
        Eigen::Quaternionf q(e_r33);
        qx=q.x();
        qy=q.y();
        qz=q.z();
        qw=q.w();


        tx=M.at<double>(0,3);
        ty=M.at<double>(1,3);
        tz=M.at<double>(2,3);
    }

    void Stereo_Inertial::savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp)
    {
        std::ofstream file(filename);
        double qx, qy, qz, qw, tx, ty, tz;
        for (auto frame : fmp)
        {
            if (!frame.second.empty())
            {
                cv::Mat minv=frame.second.inv();
                getQuaternionAndTranslationfromMatrix44(minv, qx, qy, qz, qw, tx, ty, tz);
                file << frame.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
                     << qw << endl;
            }
        }
    }

//    void LoadCameraPose(const cv::Mat &Tcw)
//    {
//        //转置
//        if(!Tcw.empty())
//        {
//            pangolin::OpenGlMatrix M;
//
//            M.m[0] = Tcw.at<float>(0,0);
//            M.m[1] = Tcw.at<float>(1,0);
//            M.m[2] = Tcw.at<float>(2,0);
//            M.m[3]  = 0.0;
//
//            M.m[4] = Tcw.at<float>(0,1);
//            M.m[5] = Tcw.at<float>(1,1);
//            M.m[6] = Tcw.at<float>(2,1);
//            M.m[7]  = 0.0;
//
//            M.m[8] = Tcw.at<float>(0,2);
//            M.m[9] = Tcw.at<float>(1,2);
//            M.m[10] = Tcw.at<float>(2,2);
//            M.m[11]  = 0.0;
//
//            M.m[12] = Tcw.at<float>(0,3);
//            M.m[13] = Tcw.at<float>(1,3);
//            M.m[14] = Tcw.at<float>(2,3);
//            M.m[15]  = 1.0;
//            //加载到opengl堆栈
//            M.Load();
//        }
//    }

}
