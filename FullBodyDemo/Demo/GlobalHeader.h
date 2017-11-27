#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTC;

#define LEN                 200
#define BIG_NUM             100000
#define SMALL_NUM          -100000
#define SMALL_FLOAT         0.00001f
#define ZOOM_SCALE          0.111
#define LEFT_SIDE           true
#define RIGHT_SIDE          false
#define MAX_DEPTH 4000
#define MIN_DEPTH 1000
#define AVERAGE_TIMES 10                                                    /// 采集深度相机数据的次数,然后求平均

#define COE_QIANYAOJIE      (45.0f / 51.0f)
#define COE_JIANKUAN        (48.0f / 40.0f * 0.88)
#define COE_XIONGWEI        (47.0f / 38.0f)
#define COE_YAOWEI_FRONT    (52.0f / 45.0f)
#define COE_YAOWEI_DIST     (42.0f / 36.0f)
#define COE_TUNWEI_FRONT    (48.0f / 46.0f)
#define COE_TUNWEI_DIST     (44.0f / 37.0f)
#define COE_BICHANG         (55.0f / 63.0f)
#define COE_TUICHANG        (1.0f)
#define COE_XIUQUAN_FRONT   (52.0f / 25.0f)
#define COE_SHANGBIWEI      (34.0f / 13.0f)
#define COE_LINGWEI         (42.0f / 24.0)



typedef enum tagEMSide
{
    OW_EM_DEFAULT = 0,
    OW_EM_LEFT    = 1,
    OW_EM_RIGHT   = 2
}EMSide;

typedef struct tagSTSkeleton
{
    pcl::PointCloud<PointT>::Ptr vecLeftHand;
    pcl::PointCloud<PointT>::Ptr vecRightHand;
    pcl::PointCloud<PointT>::Ptr vecLeftLeg;
    pcl::PointCloud<PointT>::Ptr vecRightLeg;
    pcl::PointCloud<PointT>::Ptr vecTosor;
}STSkeleton;

typedef struct tagSTKeyPoints
{
    PointT stAxillary_left;
    PointT stAxillary_Right;
    PointT stShoulder_left;
    PointT stShoulder_Right;
    PointT stNeck_Middle;
    PointT stNeck_Left;
    PointT stNeck_Right;
    PointT stHip_Left;
    PointT stHip_Right;
    PointT stWaist_Left;
    PointT stWaist_Right;
    PointT stChest_Left;
    PointT stChest_Right;
}STKeyPoints;

typedef struct tagSTBoundingBox
{
    float fLeft;
    float fDown;
    float fRight;
    float fUp;
    float fFront;
    float fBack;
    tagSTBoundingBox()
    {
        fLeft = BIG_NUM;
        fDown = BIG_NUM;
        fBack = BIG_NUM;
        fRight = SMALL_NUM;
        fUp = SMALL_NUM;
        fFront = SMALL_NUM;
    }
}STBoundingBox;

typedef struct tagSTKeypointsFromNite
{
    PointT stHead;
    PointT stNeck;
    PointT stLeftShoulder;
    PointT stRightShoulder;
    PointT stLeftElbow;
    PointT stRightElbow;
    PointT stLeftHand;
    PointT stRightHand;
    PointT stTorso;
    PointT stLeftHip;
    PointT stRightHip;
    PointT stLeftKnee;
    PointT stRightKnee;
    PointT stLeftFoot;
    PointT stRightFoot;
}STKeypointsFromNite;

typedef struct tagSTParam
{
    float fShengao;
    float fLinwei;
    float fJiankuan;
    float fXiongkuan;
    float fYaowei;
    float fBichang;
    float fTunwei;
    float fTuichang;
    float fXiuquan;
    float fShangbiwei;
    float fQianyaojie;
}STParam;

typedef struct tagSTDataForRender
{
    cv::Mat matDepthMap;
    STBoundingBox stBoundingBox;
    int nWidth;
    int nHeight;
    float fGridStep;
    STSkeleton stSkeleton;
    STKeyPoints stKeypoints;
    STKeypointsFromNite stKeypointsFromNite;
}STDataForRender;