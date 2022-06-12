//
//  draw_result.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/16.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef draw_result_hpp
#define draw_result_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "../utility.h"
#include "delaunay.h"
#include "../log_util.h"

using namespace cv;
using namespace Eigen;
using namespace std;

#define HEIGHT 640//1280
#define WIDTH 480//720

#define RIC_y ((double)0.0)
#define RIC_p ((double)0.0)
#define RIC_r ((double)180.0)

#define TIC_X ((double)0.0)
#define TIC_Y ((double)0.092)
#define TIC_Z ((double)0.01)

//pixel
#define FOCUS_LENGTH_X ((double)904.51708984375)
#define FOCUS_LENGTH_Y ((double)903.075317382813)
#define PX ((double)638.327392578125)
#define PY ((double)355.844757080078)

#define C_PI 3.1415926
#define WINDOW_SIZE 10


struct GroundPoint
{
    int idx;
    Vector3f center;
    bool boxflag;
    bool moveflag;
    Vector3f ori, cox, coy, coz;
    Vector3f lix, liy, liz;
    float size;
    Vector4f initPlane;

    GroundPoint(int idx_, Vector3f center_)
    {
        idx = idx_;
        center = center_;
        boxflag = false;
        moveflag = false;
    }
};

class DrawResult
{
public:
    DrawResult(float _pitch, float _roll, float _yaw, float _Tx, float _Ty, float _Tz);

    float roll, pitch, yaw; //in degree
    float Tx, Ty, Tz; //in meters, only in z axis
    float radius, radiusAR;
    float theta, phy;
    float thetaAR, phyAR;
    float locationX, locationY;
    float locationX_p, locationY_p;
    float locationXT2, locationYT2;
    float locationXT2_p, locationYT2_p;
    float locationXP, locationYP;
    float locationXP_p, locationYP_p;
    float locationTapX, locationTapY;
    bool tapFlag;
    float locationLongPressX;
    float locationLongPressY;
    bool longPressFlag;


    float theta_p, phy_p, radius_p;
    float X0_p, Y0_p;
    int finger_state;
    int finger_s;
    int finger_d;
    int finger_p;

    //map<int, pair <int, pair< Vector3f, vector<Vector3f> > > > Ground;
    //map<int, pair <int, Vector3f > > Ground;
    vector<GroundPoint> Grounds;

    int Ground_idx;

    float lengthCube;
    vector<Vector3f> pose;
    vector<int> segment_indexs;
    float Fx,Fy;
    bool change_view_manualy;
    bool planeInit;
    bool startInit;
    Vector3f initPoint;
    Vector4f initPlane;
    Vector3f origin;
    Vector3f ConerX, ConerY, ConerZ;
    Vector3f lineX,lineY,lineZ;
    Vector3f origin_w;
    float X0, Y0;
    float X0AR, Y0AR;

    bool look_down;
    //for optical flow EKF
    cv::Mat pre_image;
    cv::Mat cur_image;
    vector<Point2f> pre_pts;
    vector<Point2f> cur_pts;
    vector<Point2f> n_pts;
    Point2f flow;  //velocity of the pixel
    bool KF_init;

    vector<Scalar> trajectory_color;
    vector<int> indexs;
    bool change_color;
    //4 sizes
    vector<Vector2f> pre_status, cur_status;
    vector<Matrix2f> K;
    vector<Matrix2f> cur_cov, pre_cov;
    void computeAR(vector<Vector3f> &point_cloud, Vector3f &model);

    void drawAR(cv::Mat &result, vector<Vector3f> &point_cloud, Vector3f P_latest, Matrix3f R_latest);
    void drawGround(cv::Mat &result, vector<Vector3f> &point_cloud, Vector3f P_latest, Matrix3f R_latest);
    void drawBox(cv::Mat &result, const Vector3f& corner_0, const Vector3f& corner_x, const Vector3f& corner_y, Vector3f corner_z, float size, Vector3f P_latest, Matrix3f R_latest, bool inAR) const;
    void Reprojection(cv::Mat &result, vector<Vector3f> &point_cloud, const Matrix3f *R_window,const Vector3f *T_window, bool box_in_trajectory);
    static vector<Vector3f> calculate_camera_pose(const Vector3f& camera_center, Matrix3f Rc, float length);
    cv::Point2f World2VirturCam(const Eigen::Vector3f& xyz, float &depth);
    void drawBoxVirturCam(cv::Mat &result);
    void rejectWithF();
    static cv::Scalar newColor();

    //add
    void drawPose(cv::Mat& result,const Matrix3f& r,const Vector3f& curTrans);
    Vector3f lastTrans{0.0 , 0.0 , 0.0};
private:
    static Vector3f findZfromXY(Vector3f point, Vector4f plane);
    Vector4f findPlane(vector<Vector3f> &point_cloud);
    Vector3f findGround(vector<Vector3f> &point_cloud, vector<Vector3f> &inlier_points) const;

};

#endif /* draw_result_hpp */
