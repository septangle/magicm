#pragma once

#include "GlobalHeader.h"
#include "Render.h"

class CMeasure
{
public:
    CMeasure();
    ~CMeasure();
    int Init(vector<cv::Point3f>& vecBodyPoints, STKeypointsFromNite& stKeyPointsFromNite);
    int StartMeasure(STParam& stParam);
    int GetDataForRender(STDataForRender& stDataForRender);

private:
    int GetBoudingBox();
    int GetDepthMap();
    int GetSkeleton();
    int LocateKeyPoints();

    int FindAxillary(pcl::PointCloud<PointT>::Ptr pLeftHand, pcl::PointCloud<PointT>::Ptr pRightHand, EMSide emSide);
    int FindShoulder(EMSide emSide);
    int FindNeckMiddle();
    int FindNeckSide(EMSide emSide);
    int FindHip(PointT& stHipFromNiteLeft, PointT& stHipFromNiteRight, EMSide emSide);
    int FindWaist(EMSide emSide);
    int FindChest(EMSide emSide);

private:
    int GetParam(STParam& stParam);
    int GetShenggao(float& fShengao);
    int GetQianyaojie(float& fQianyaojie);
    int GetJiankuan(float& fJiankuan);
    int GetXiongkuan(float& fXiongkuan);
    int GetYaowei(float& fYaowei);
    int GetTunwei(float& fTunwei);
    int GetBichang(float& fBichang);
    int GetTuichang(float& fTuichang);
    int GetXiuquan(float& fXiuquan);
    int GetShangbiwei(float& fShangbiwei);
    int GetLingwei(float& fLingwei);

    float CalcDistHorizon(PointT& stStart, PointT& stEnd);
    float CalcDistVertical(PointT& stStart, PointT& stEnd);
    float CalcDistOblique(PointT& stPoint, PointT& stNormal, EMSide emSide);

    float CalDist(float x, float y, float z, float x1, float y1, float z1);
    float CalDist(PointT& point1, PointT& point2);
    void AdjustPoint(PointT& point, float fx, float fy, float fz);

private:
    pcl::PointCloud<PointT>::Ptr m_pPointInput;
    pcl::PointCloud<PointT>::Ptr m_pPointSmooth;
    pcl::PointCloud<PointT>::Ptr m_pSkeleton;
    STBoundingBox m_stBoundingBox;
    cv::Mat m_matDepthMap;
    int     m_nWidthLen;
    float   m_fGridStep;
    CRender* m_pRender;
    STKeypointsFromNite m_stKeyPointsFromNite;
    STKeyPoints m_stKeyPoints;
    STSkeleton m_stSkeleton;
};

