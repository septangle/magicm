#include "Measure.h"
#include "Math.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/convolution.h>
#include <pcl/filters/convolution_3d.h>

/************************************************************************/
/* PointT加减乘除的重载                                                  */
/************************************************************************/
PointT operator+(PointT& point1, PointT& point2)
{
    PointT point;
    point.x = point1.x + point2.x;
    point.y = point1.y + point2.y;
    point.z = point1.z + point2.z;
    return point;
}

PointT operator-(PointT& point1, PointT& point2)
{
    PointT point;
    point.x = point1.x - point2.x;
    point.y = point1.y - point2.y;
    point.z = point1.z - point2.z;
    return point;
}

PointT operator * (PointT& point1, float fValue)
{
    PointT point;
    point.x = point1.x * fValue;
    point.y = point1.y * fValue;
    point.z = point1.z * fValue;
    return point;
}

PointT operator / (PointT& point1, float fValue)
{
    PointT point;
    point.x = point1.x / fValue;
    point.y = point1.y / fValue;
    point.z = point1.z / fValue;
    return point;
}

CMeasure::CMeasure()
{
    m_nWidthLen = 0;
    m_fGridStep = 0;
    pcl::PointCloud<PointT>::Ptr tempPtr(new pcl::PointCloud<PointT>);
    m_pPointInput = tempPtr;

    pcl::PointCloud<PointT>::Ptr tempPtr1(new pcl::PointCloud<PointT>);
    m_pPointSmooth = tempPtr1;

    pcl::PointCloud<PointT>::Ptr tempPtr2(new pcl::PointCloud<PointT>);
    m_pSkeleton = tempPtr2;
}

CMeasure::~CMeasure()
{
}

int CMeasure::Init(vector<cv::Point3f>& vecBodyPoints, STKeypointsFromNite& stKeyPointsFromNite)
{
    m_pPointInput->clear();
    m_pPointSmooth->clear();
    m_pSkeleton->clear();

    for (int i = 0 ; i < vecBodyPoints.size() ; i++)
    {
        PointT stPoint;
        stPoint.x = vecBodyPoints.at(i).x;
        stPoint.y = vecBodyPoints.at(i).y;
        stPoint.z = vecBodyPoints.at(i).z;

        m_pPointInput->push_back(stPoint);
        m_pPointSmooth->push_back(stPoint);
    }

    memcpy(&m_stKeyPointsFromNite, &stKeyPointsFromNite, sizeof(m_stKeyPointsFromNite));

//     ofstream stream("debugfile.txt", ios::out);
//     stream << m_stKeyPointsFromNite.stHead << " " << endl;
//     stream << m_stKeyPointsFromNite.stLeftElbow << " " << endl;
//     stream << m_stKeyPointsFromNite.stLeftFoot << " " << endl;
//     stream << m_stKeyPointsFromNite.stLeftHand << " " << endl;
//     stream << m_stKeyPointsFromNite.stLeftHip << " " << endl;
//     stream << m_stKeyPointsFromNite.stLeftKnee << " " << endl;
//     stream << m_stKeyPointsFromNite.stLeftShoulder << " " << endl;
//     stream << m_stKeyPointsFromNite.stNeck << " " << endl;
//     stream << m_stKeyPointsFromNite.stRightElbow << " " << endl;
//     stream << m_stKeyPointsFromNite.stRightFoot << " " << endl;
//     stream << m_stKeyPointsFromNite.stRightHand << " " << endl;
//     stream << m_stKeyPointsFromNite.stRightHip << " " << endl;
//     stream << m_stKeyPointsFromNite.stRightKnee << " " << endl;
// 
//     for (int i = 0; i < vecBodyPoints.size(); i++)
//     {
//         PointT stPoint;
//         stPoint.x = vecBodyPoints.at(i).x;
//         stPoint.y = vecBodyPoints.at(i).y;
//         stPoint.z = vecBodyPoints.at(i).z;
// 
//         stream << stPoint.x << " " << stPoint.y << " " << stPoint.z << endl;
//     }

    return 0;
}

int CMeasure::StartMeasure(STParam& stParam)
{
    int nResult = 0;

    GetBoudingBox();
    GetDepthMap();

    nResult = GetSkeleton();
    if (nResult == -1)
    {
        return -1;
    }

    LocateKeyPoints();

    GetParam(stParam);

    return 0;
}

void CMeasure::AdjustPoint(PointT& point, float fx, float fy, float fz)
{
    point.x += fx;
    point.y += fy;
    point.z += fz;
}

int CMeasure::GetBoudingBox()
{
    for (int i = 0 ; i < m_pPointSmooth->size() ; i++)
    {
        PointT stPoint = m_pPointSmooth->at(i);
        if (stPoint.x > m_stBoundingBox.fRight)
        {
            m_stBoundingBox.fRight = stPoint.x;
        }

        if (stPoint.x < m_stBoundingBox.fLeft)
        {
            m_stBoundingBox.fLeft = stPoint.x;
        }

        if (stPoint.y > m_stBoundingBox.fUp)
        {
            m_stBoundingBox.fUp = stPoint.y;
        }

        if (stPoint.y < m_stBoundingBox.fDown)
        {
            m_stBoundingBox.fDown = stPoint.y;
        }

        if (stPoint.z > m_stBoundingBox.fFront)
        {
            m_stBoundingBox.fFront = stPoint.z;
        }

        if (stPoint.z < m_stBoundingBox.fBack)
        {
            m_stBoundingBox.fBack = stPoint.z;
        }
    }

    PointT stOffset(
        (m_stBoundingBox.fLeft + m_stBoundingBox.fRight) / 2,
        (m_stBoundingBox.fDown + m_stBoundingBox.fUp) / 2,
        (m_stBoundingBox.fBack + m_stBoundingBox.fFront) / 2);

    PointT stCenter;
    for (int i = 0; i < m_pPointSmooth->size(); i++)
    {
        stCenter.x += m_pPointSmooth->at(i).x;
        stCenter.y += m_pPointSmooth->at(i).y;
        stCenter.z += m_pPointSmooth->at(i).z;
    }

    stCenter.x = stCenter.x / m_pPointSmooth->size();
    stCenter.y = stCenter.y / m_pPointSmooth->size();
    stCenter.z = stCenter.z / m_pPointSmooth->size();

    for (int i = 0 ; i < m_pPointSmooth->size() ; i++)
    {
        m_pPointSmooth->at(i).x -= stCenter.x;
        m_pPointSmooth->at(i).y -= stCenter.y;
        m_pPointSmooth->at(i).z -= stCenter.z;
    }

    m_stBoundingBox.fLeft -= stCenter.x;
    m_stBoundingBox.fRight -= stCenter.x;
    m_stBoundingBox.fDown -= stCenter.y;
    m_stBoundingBox.fUp -= stCenter.y;
    m_stBoundingBox.fBack -= stCenter.z;
    m_stBoundingBox.fFront -= stCenter.z;

    AdjustPoint(m_stKeyPointsFromNite.stHead, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stLeftElbow, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stLeftFoot, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stLeftHand, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stLeftHip, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stLeftKnee, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stLeftShoulder, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stNeck, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stRightElbow, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stRightFoot, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stRightHand, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stRightHip, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stRightKnee, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stRightShoulder, -stCenter.x, -stCenter.y, -stCenter.z);
    AdjustPoint(m_stKeyPointsFromNite.stTorso, -stCenter.x, -stCenter.y, -stCenter.z);

    float fRate = (m_stBoundingBox.fRight - m_stBoundingBox.fLeft) /
        (m_stBoundingBox.fUp - m_stBoundingBox.fDown);

    m_nWidthLen = LEN * fRate;
    m_fGridStep = (float)(m_stBoundingBox.fUp - m_stBoundingBox.fDown) / LEN;

    m_matDepthMap.create(LEN + 1, m_nWidthLen + 1, CV_32FC1);

    for (int i = 0 ; i < LEN ; i++)
    {
        for (int j = 0 ; j < m_nWidthLen ; j++)
        {
            m_matDepthMap.at<float>(i, j) = SMALL_NUM;
        }
    }

    return 0;
}

int CMeasure::GetDepthMap()
{
    int nPosX = 0;
    int nPosY = 0;
    for (int i = 0; i < m_pPointSmooth->size(); i++)
    {
        PointT pointT = m_pPointSmooth->at(i);
        nPosX = (pointT.x - m_stBoundingBox.fLeft) / m_fGridStep;
        nPosY = (pointT.y - m_stBoundingBox.fDown) / m_fGridStep;
        if (pointT.z > m_matDepthMap.at<float>(nPosY, nPosX))
        {
            m_matDepthMap.at<float>(nPosY, nPosX) = pointT.z;
        }
    }

    cv::Mat matTemp = m_matDepthMap.clone();

    for (int i = 0; i < LEN; i++)
    {
        for (int j = 0 ; j < m_nWidthLen ; j++)
        {
            if (i - 1 < 0 || i + 1 >= LEN || j - 1 < 0 || j + 1 >= LEN)
            {
                continue;
            }

            int nCount = 0;
            for (int m = -1; m < 2; m++)
            {
                for (int n = -1; n < 2; n++)
                {
                    if (m == 0 && n == 0)
                    {
                        continue;
                    }

                    if (matTemp.at<float>(i + m, j + n))
                    {
                        nCount++;
                    }
                }
            }

            if (nCount < 2)
            {
                m_matDepthMap.at<float>(i, j) = SMALL_NUM;
            }
        }
    }

    return 0;
}

int CMeasure::GetDataForRender(STDataForRender& stDataForRender)
{
    stDataForRender.matDepthMap = m_matDepthMap;
    stDataForRender.stBoundingBox = m_stBoundingBox;
    stDataForRender.nWidth = m_nWidthLen;
    stDataForRender.nHeight = LEN;
    stDataForRender.fGridStep = m_fGridStep;
    stDataForRender.stSkeleton = m_stSkeleton;
    stDataForRender.stKeypoints = m_stKeyPoints;
    stDataForRender.stKeypointsFromNite = m_stKeyPointsFromNite;

    return 0;
}

int CMeasure::GetSkeleton()
{
    /************************************************************************/
    /* 骨架寻找                                                              */
    /************************************************************************/
    PointT pointLeft;
    PointT pointRight;
    PointT pointCenter;
    pcl::PointCloud<PointT>::Ptr pSkeletonOriginal;
    for (int i = LEN - 1; i > 0; i--)
    {
        bool bFlag = false;
        for (int j = 0; j < m_nWidthLen; j++)
        {
            float fDepth = m_matDepthMap.at<float>(i, j);
            if (bFlag == false && fDepth < SMALL_NUM + 1)
            {
                continue;
            }

            if (bFlag == false && fDepth > SMALL_NUM + 1)
            {
                pointLeft.x = j * m_fGridStep + m_stBoundingBox.fLeft;
                pointLeft.y = i * m_fGridStep + m_stBoundingBox.fDown;
                pointLeft.z = fDepth;
                bFlag = true;
            }

            if (bFlag == true && fDepth > SMALL_NUM + 1)
            {
                pointRight.x = j * m_fGridStep + m_stBoundingBox.fLeft;
                pointRight.y = i * m_fGridStep + m_stBoundingBox.fDown;
                pointRight.z = fDepth;
            }

            if ((bFlag == true && fDepth < SMALL_NUM + 1) || j == m_nWidthLen - 1)
            {
                pointCenter = PointT(
                    (pointLeft.x + pointRight.x) / 2,
                    (pointLeft.y + pointRight.y) / 2,
                    (pointLeft.z + pointRight.z) / 2);
                bFlag = false;

                if ((pointRight.x - pointLeft.x) / m_fGridStep > 2)
                {
                    m_pSkeleton->push_back(pointCenter);
                }
            }
        }
    }

    int nSkeletonLineNum = 5;
    pcl::PointCloud<PointT>::Ptr* szSkeleton = new pcl::PointCloud<PointT>::Ptr[nSkeletonLineNum];
    for (int i = 0; i < nSkeletonLineNum; i++)
    {
        pcl::PointCloud<PointT>::Ptr ptrTmp(new pcl::PointCloud<PointT>);
        szSkeleton[i] = ptrTmp;
    }

    for (int i = 0; i < nSkeletonLineNum; i++)
    {
        if (m_pSkeleton->size() == 0)
        {
            break;
        }
        int nCount = 0;
        pcl::PointCloud<PointT>::iterator iterMain = m_pSkeleton->begin();
        while (iterMain != m_pSkeleton->end())
        {
            if (szSkeleton[i]->size() == 0)
            {
                szSkeleton[i]->push_back(*iterMain);
                iterMain = m_pSkeleton->erase(iterMain);
                continue;
            }

            PointT pointOld = szSkeleton[i]->back();
            PointT pointNew = *iterMain;
            float fDist = (pointOld.x - pointNew.x) * (pointOld.x - pointNew.x) + (pointOld.y - pointNew.y) * (pointOld.y - pointNew.y);
            if (fDist < m_fGridStep * m_fGridStep * 10 * 10)
            {
                szSkeleton[i]->push_back(*iterMain);
                iterMain = m_pSkeleton->erase(iterMain);
                nCount = 0;
                continue;
            }
            else
            {
                if (iterMain == m_pSkeleton->end())
                {
                    break;
                }
                iterMain++;
                nCount++;
                if (nCount > 10)
                {
                    break;
                }
            }
        }

        if (szSkeleton[i]->size() < 10)
        {
            szSkeleton[i]->clear();
            i--;
        }
    }

    if (szSkeleton[0]->size() == 0 ||
        szSkeleton[1]->size() == 0 ||
        szSkeleton[2]->size() == 0 ||
        szSkeleton[3]->size() == 0 ||
        szSkeleton[4]->size() == 0 )
    {
        return -1;
    }

    m_stSkeleton.vecTosor = szSkeleton[0];

    if (szSkeleton[1]->at(0).x < szSkeleton[2]->at(0).x)
    {
        m_stSkeleton.vecLeftHand = szSkeleton[1];
        m_stSkeleton.vecRightHand = szSkeleton[2];
    }
    else
    {
        m_stSkeleton.vecLeftHand = szSkeleton[2];
        m_stSkeleton.vecRightHand = szSkeleton[1];
    }

    if (szSkeleton[3]->at(0).x < szSkeleton[4]->at(0).x)
    {
        m_stSkeleton.vecLeftLeg = szSkeleton[3];
        m_stSkeleton.vecRightLeg = szSkeleton[4];
    }
    else
    {
        m_stSkeleton.vecLeftLeg = szSkeleton[3];
        m_stSkeleton.vecRightLeg = szSkeleton[4];
    }

    return 0;
}

int CMeasure::LocateKeyPoints()
{
    FindAxillary(m_stSkeleton.vecLeftHand, m_stSkeleton.vecRightHand, OW_EM_LEFT);
    FindAxillary(m_stSkeleton.vecLeftHand, m_stSkeleton.vecRightHand, OW_EM_RIGHT);
    FindShoulder(OW_EM_LEFT);
    FindShoulder(OW_EM_RIGHT);
    FindNeckMiddle();
    FindNeckSide(OW_EM_LEFT);
    FindNeckSide(OW_EM_RIGHT);
    FindHip(m_stKeyPointsFromNite.stLeftHip, m_stKeyPointsFromNite.stRightHip, OW_EM_LEFT);
    FindHip(m_stKeyPointsFromNite.stLeftHip, m_stKeyPointsFromNite.stRightHip, OW_EM_RIGHT);
    FindWaist(OW_EM_LEFT);
    FindWaist(OW_EM_RIGHT);
    FindChest(OW_EM_LEFT);
    FindChest(OW_EM_RIGHT);

    return 0;
}

int CMeasure::FindAxillary(pcl::PointCloud<PointT>::Ptr pLeftHand, pcl::PointCloud<PointT>::Ptr pRightHand, EMSide emSide)
{
    PointT pointEndLeft  = pLeftHand->at(0);
    PointT pointEndRight = pRightHand->at(0);

    int nPosX = 0;
    int nPosY = 0;

    nPosX = ((pointEndLeft.x + pointEndRight.x) / 2 - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;

    if (emSide == OW_EM_LEFT)
    {
        nPosY = (pointEndLeft.y - m_stBoundingBox.fDown) / m_fGridStep;
    }
    else
    {
        nPosY = (pointEndRight.y - m_stBoundingBox.fDown) / m_fGridStep;
    }

    PointT pointTmp;
    pointTmp.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
    pointTmp.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
    pointTmp.z = m_matDepthMap.at<float>(nPosY, nPosX);

    while (true)
    {
        if (pointTmp.z < SMALL_NUM + 1)
        {
            break;
        }

        if (emSide == OW_EM_LEFT)
        {
            m_stKeyPoints.stAxillary_left = pointTmp;
            nPosX--;
        }
        else
        {
            m_stKeyPoints.stAxillary_Right = pointTmp;
            nPosX++;
        }

        pointTmp.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
        pointTmp.z = m_matDepthMap.at<float>(nPosY, nPosX);
    }

    return 0;
}

int CMeasure::FindShoulder(EMSide emSide)
{
    int nPosX = 0;
    int nPosY = 0;

    if (emSide == OW_EM_LEFT)
    {
        nPosX = (m_stKeyPoints.stAxillary_left.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
        nPosY = (m_stKeyPoints.stAxillary_left.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;
    }
    else
    {
        nPosX = (m_stKeyPoints.stAxillary_Right.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
        nPosY = (m_stKeyPoints.stAxillary_Right.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;
    }

    PointT pointTmp;
    pointTmp.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
    pointTmp.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
    pointTmp.z = m_matDepthMap.at<float>(nPosY, nPosX);

    while (true)
    {
        if (pointTmp.z < SMALL_NUM + 1)
        {
            break;
        }

        if (emSide == OW_EM_LEFT)
        {
            m_stKeyPoints.stShoulder_left = pointTmp;
        }
        else
        {
            m_stKeyPoints.stShoulder_Right = pointTmp;
        }

        nPosY++;
        pointTmp.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
        pointTmp.z = m_matDepthMap.at<float>(nPosY, nPosX);
    }

    return 0;
}

int CMeasure::FindNeckMiddle()
{
    int nPosX1 = (m_stKeyPoints.stShoulder_left.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY1 = (m_stKeyPoints.stShoulder_left.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;

    int nPosX2 = (m_stKeyPoints.stShoulder_Right.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY2 = (m_stKeyPoints.stShoulder_Right.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;

    int nPosX = (nPosX1 + nPosX2) / 2;
    int nPosY = (nPosY1 + nPosY2) / 2;

    m_stKeyPoints.stNeck_Middle.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
    m_stKeyPoints.stNeck_Middle.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
    m_stKeyPoints.stNeck_Middle.z = m_matDepthMap.at<float>(nPosY, nPosX);

    return 0;
}

int CMeasure::FindNeckSide(EMSide emSide)
{
    int nPosX1 = (m_stKeyPoints.stNeck_Middle.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY1 = (m_stKeyPoints.stNeck_Middle.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;

    int nPosX = nPosX1;
    int nPosY = nPosY1 + 6;                                          ///< 常数为暂定值
    float fPosZ = m_matDepthMap.at<float>(nPosY, nPosX);

    ///< 搜索左颈侧点
    while (true)
    {
        if (fPosZ < SMALL_NUM + 1)
        {
            break;
        }

        if (emSide == OW_EM_LEFT)
        {
            m_stKeyPoints.stNeck_Left.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stNeck_Left.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stNeck_Left.z = fPosZ;
            nPosX--;
        }
        else
        {
            m_stKeyPoints.stNeck_Right.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stNeck_Right.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stNeck_Right.z = fPosZ;
            nPosX++;
        }

        fPosZ = m_matDepthMap.at<float>(nPosY, nPosX);
    }

    return 0;
}

int CMeasure::FindHip(PointT& stHipFromNiteLeft, PointT& stHipFromNiteRight, EMSide emSide)
{
    float fX = (stHipFromNiteRight.x + stHipFromNiteLeft.x) / 2;
    float fY = (stHipFromNiteRight.y + stHipFromNiteLeft.y) / 2;
    int nPosX = (fX - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY = (fY - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;
    float fZ = m_matDepthMap.at<float>(nPosY, nPosX);

    while (nPosX >= 0 && nPosY < m_nWidthLen)
    {
        if (fZ < SMALL_NUM + 1)
        {
            break;
        }

        if (emSide == OW_EM_LEFT)
        {
            m_stKeyPoints.stHip_Left.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stHip_Left.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stHip_Left.z = fZ;
            nPosX--;
        }
        else
        {
            m_stKeyPoints.stHip_Right.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stHip_Right.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stHip_Right.z = fZ;
            nPosX++;
        }

        fZ = m_matDepthMap.at<float>(nPosY, nPosX);
    }

    return 0;
}

int CMeasure::FindWaist(EMSide emSide)
{
    ///< 髋中心点平均位置
    float fPosX1 = (m_stKeyPoints.stHip_Left.x + m_stKeyPoints.stHip_Right.x) / 2;
    float fPosY1 = (m_stKeyPoints.stHip_Left.y + m_stKeyPoints.stHip_Right.y) / 2;

    ///< 肩中心点平均位置
    float fPosX2 = (m_stKeyPoints.stShoulder_left.x + m_stKeyPoints.stShoulder_Right.x) / 2;
    float fPosY2 = (m_stKeyPoints.stShoulder_left.y + m_stKeyPoints.stShoulder_Right.y) / 2;

    ///< 腰在髋与肩膀之间的系数
    float fCoe = 0.2f;
    float fPosX = fPosX1 + (fPosX2 - fPosX1) * fCoe;
    float fPosY = fPosY1 + (fPosY2 - fPosY1) * fCoe;

    int nPosX = (fPosX - m_stBoundingBox.fLeft) / m_fGridStep;
    int nPosY = (fPosY - m_stBoundingBox.fDown) / m_fGridStep;
    float fZ = m_matDepthMap.at<float>(nPosY, nPosX);

    while (true)
    {
        if (fZ < SMALL_NUM + 1)
        {
            break;
        }

        if (emSide == OW_EM_LEFT)
        {
            m_stKeyPoints.stWaist_Left.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stWaist_Left.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stWaist_Left.z = fZ;
            nPosX--;
        }
        else
        {
            m_stKeyPoints.stWaist_Right.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stWaist_Right.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stWaist_Right.z = fZ;
            nPosX++;
        }

        fZ = m_matDepthMap.at<float>(nPosY, nPosX);
    }

    return 0;
}

int CMeasure::FindChest(EMSide emSide)
{
    ///< 髋中心点平均位置
    float fPosX1 = (m_stKeyPoints.stHip_Left.x + m_stKeyPoints.stHip_Right.x) / 2;
    float fPosY1 = (m_stKeyPoints.stHip_Left.y + m_stKeyPoints.stHip_Right.y) / 2;

    ///< 肩中心点平均位置
    float fPosX2 = (m_stKeyPoints.stShoulder_left.x + m_stKeyPoints.stShoulder_Right.x) / 2;
    float fPosY2 = (m_stKeyPoints.stShoulder_left.y + m_stKeyPoints.stShoulder_Right.y) / 2;

    ///< 胸在髋与肩膀之间的系数
    float fCoe = 0.6f;
    float fPosX = fPosX1 + (fPosX2 - fPosX1) * fCoe;
    float fPosY = fPosY1 + (fPosY2 - fPosY1) * fCoe;

    int nPosX = (fPosX - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY = (fPosY - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;
    float fZ = m_matDepthMap.at<float>(nPosY, nPosX);

    while (true)
    {
        if (fZ < SMALL_NUM + 1)
        {
            break;
        }

        float fxTmp = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
        float fyTmp = nPosY * m_fGridStep + m_stBoundingBox.fDown;
        float fZTmp = fZ;

        if (fxTmp < m_stKeyPoints.stAxillary_left.x ||
            fxTmp > m_stKeyPoints.stAxillary_Right.x)
        {
            break;
        }

        if (emSide == OW_EM_LEFT)
        {
            m_stKeyPoints.stChest_Left.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stChest_Left.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stChest_Left.z = fZ;
            nPosX--;
        }
        else
        {
            m_stKeyPoints.stChest_Right.x = nPosX * m_fGridStep + m_stBoundingBox.fLeft;
            m_stKeyPoints.stChest_Right.y = nPosY * m_fGridStep + m_stBoundingBox.fDown;
            m_stKeyPoints.stChest_Right.z = fZ;
            nPosX++;
        }

        fZ = m_matDepthMap.at<float>(nPosY, nPosX);
    }

    return 0;
}

int CMeasure::GetParam(STParam& stParam)
{
    GetShenggao(stParam.fShengao);
    GetQianyaojie(stParam.fQianyaojie);
    GetJiankuan(stParam.fJiankuan);
    GetXiongkuan(stParam.fXiongkuan);
    GetYaowei(stParam.fYaowei);
    GetTunwei(stParam.fTunwei);
    GetBichang(stParam.fBichang);
    GetTuichang(stParam.fTuichang);
    GetXiuquan(stParam.fXiuquan);
    GetShangbiwei(stParam.fShangbiwei);
    GetLingwei(stParam.fLinwei);

    return 0;
}

int CMeasure::GetShenggao(float& fShengao)
{
    float fYMax = SMALL_NUM;
    float fYMin = BIG_NUM;

    for (int i = 0 ; i < m_matDepthMap.size().height ; i++)
    {
        for (int j = 0; j < m_matDepthMap.size().width; j++)
        {
            float fZ = m_matDepthMap.at<float>(i, j);
            if (fZ < SMALL_NUM + 1)
            {
                continue;
            }

            float fY = i * m_fGridStep + m_stBoundingBox.fDown;
            if (fY > fYMax)
            {
                fYMax = fY;
            }

            if (fY < fYMin)
            {
                fYMin = fY;
            }
        }
    }

    fShengao = fYMax - fYMin;

    return 0;
}

float CMeasure::CalDist(float x, float y, float z, float x1, float y1, float z1)
{
    return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1) + (z - z1) * (z - z1));
}

float CMeasure::CalDist(PointT& point1, PointT& point2)
{
    return CalDist(point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
}

int CMeasure::GetQianyaojie(float& fQianyaojie)
{
    float fValue_left = CalcDistVertical(m_stKeyPoints.stNeck_Left, m_stKeyPoints.stWaist_Left);
    float fValue_right = CalcDistVertical(m_stKeyPoints.stNeck_Right, m_stKeyPoints.stWaist_Right);

    fQianyaojie = (fValue_left + fValue_right) / 2.0f;

    fQianyaojie *= COE_QIANYAOJIE;

    return 0;
}

int CMeasure::GetJiankuan(float& fJiankuan)
{
    fJiankuan = CalcDistHorizon(m_stKeyPoints.stShoulder_left, m_stKeyPoints.stShoulder_Right);

    fJiankuan = fJiankuan * COE_JIANKUAN;

    return 0;
}

int CMeasure::GetXiongkuan(float& fXiongkuan)
{
    fXiongkuan = CalcDistHorizon(m_stKeyPoints.stChest_Left, m_stKeyPoints.stChest_Right);

    float fDist = m_stKeyPoints.stChest_Right.x - m_stKeyPoints.stChest_Left.x;

    fDist = fDist * COE_XIONGWEI;

    fXiongkuan = fXiongkuan + fDist;

    return 0;
}

int CMeasure::GetYaowei(float& fYaowei)
{
    fYaowei = CalcDistHorizon(m_stKeyPoints.stWaist_Left, m_stKeyPoints.stWaist_Right);

    float fDist = m_stKeyPoints.stWaist_Right.x - m_stKeyPoints.stWaist_Left.x;

    fYaowei = fYaowei * COE_YAOWEI_FRONT;

    fYaowei = fYaowei + fDist * COE_YAOWEI_DIST;

    return 0;
}

float CMeasure::CalcDistHorizon(PointT& stStart, PointT& stEnd)
{
    float fResult = 0.0f;

    PointT stStartTmp = stStart;
    PointT stEndTmp = stEnd;

    ///< 保证开始点在结束点左边
    if (stStartTmp.x > stEnd.x)
    {
        PointT stTmp = stEndTmp;
        stEndTmp = stStartTmp;
        stStartTmp = stTmp;
    }

    int nPosX_Now = (stStartTmp.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY_Now = (stStartTmp.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;

    float fPosX_Now = stStartTmp.x;
    float fPosY_Now = stStartTmp.y;
    float fPosZ_Now = m_matDepthMap.at<float>(nPosY_Now, nPosX_Now);

    int nPosX_End = (stEndTmp.x - m_stBoundingBox.fLeft) / m_fGridStep;

    int nPosX_Next = nPosX_Now;
    int nPosY_Next = nPosY_Now;
    float fPosY_Next = nPosY_Next * m_fGridStep + m_stBoundingBox.fDown;

    while (true)
    {
        nPosX_Next++;

        float fPosZ_Next = m_matDepthMap.at<float>(nPosY_Next, nPosX_Next);

        if (nPosX_Next > nPosX_End)
        {
            break;
        }

        if (fPosZ_Next < SMALL_NUM + 1)
        {
            continue;
        }

        float fPosX_Next = nPosX_Next * m_fGridStep + m_stBoundingBox.fLeft;

        fResult += CalDist(fPosX_Next, fPosY_Next, fPosZ_Next, fPosX_Now, fPosY_Now, fPosZ_Now);

        nPosX_Now = nPosX_Next;
        nPosY_Now = nPosY_Next;
        fPosX_Now = fPosX_Next;
        fPosY_Now = fPosY_Next;
        fPosZ_Now = fPosZ_Next;
    }

    return fResult;
}

float CMeasure::CalcDistVertical(PointT& stStart, PointT& stEnd)
{
    float fResult = 0.0f;

    PointT stStartTmp = stStart;
    PointT stEndTmp = stEnd;

    ///< 保证开始点在结束点左边
    if (stStartTmp.y < stEnd.x)
    {
        PointT stTmp = stEndTmp;
        stEndTmp = stStartTmp;
        stStartTmp = stTmp;
    }

    int nPosX_Now = (stStartTmp.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY_Now = (stStartTmp.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;

    float fPosX_Now = stStartTmp.x;
    float fPosY_Now = stStartTmp.y;
    float fPosZ_Now = m_matDepthMap.at<float>(nPosY_Now, nPosX_Now);

    int nPosY_End = (stEndTmp.y - m_stBoundingBox.fDown) / m_fGridStep;

    int nPosX_Next = nPosX_Now;
    int nPosY_Next = nPosY_Now;
    float fPosX_Next = nPosX_Next * m_fGridStep + m_stBoundingBox.fLeft;

    while (true)
    {
        nPosY_Next--;
        float fPosZ_Next = m_matDepthMap.at<float>(nPosY_Next, nPosX_Next);

        if (nPosY_Next < nPosY_End)
        {
            break;
        }

        if (fPosZ_Next < SMALL_NUM + 1)
        {
            continue;
        }

        float fPosY_Next = nPosY_Next * m_fGridStep + m_stBoundingBox.fDown;
        fResult += CalDist(fPosX_Next, fPosY_Next, fPosZ_Next, fPosX_Now, fPosY_Now, fPosZ_Now);

        nPosX_Now = nPosX_Next;
        nPosY_Now = nPosY_Next;
        fPosX_Now = fPosX_Next;
        fPosY_Now = fPosY_Next;
        fPosZ_Now = fPosZ_Next;
    }

    return fResult;
}

int CMeasure::GetTunwei(float& fTunwei)
{
    fTunwei = CalcDistHorizon(m_stKeyPoints.stHip_Left, m_stKeyPoints.stHip_Right);

    fTunwei = fTunwei * COE_TUNWEI_FRONT;

    float fDist = m_stKeyPoints.stHip_Right.x - m_stKeyPoints.stHip_Left.x;

    fTunwei = fTunwei + fDist * COE_TUNWEI_DIST;

    return fTunwei;
}

int CMeasure::GetBichang(float& fBichang)
{
    float fLenShangbi_Left = CalDist(m_stKeyPoints.stShoulder_left, m_stKeyPointsFromNite.stLeftElbow);
    float fLenXiabi_left = CalDist(m_stKeyPointsFromNite.stLeftElbow, m_stKeyPointsFromNite.stLeftHand);
    float fLenShangbi_Right = CalDist(m_stKeyPoints.stShoulder_Right, m_stKeyPointsFromNite.stRightElbow);
    float fLenXiabi_Right = CalDist(m_stKeyPointsFromNite.stRightElbow, m_stKeyPointsFromNite.stRightHand);

    fBichang = (fLenShangbi_Left + fLenXiabi_left + fLenShangbi_Right + fLenXiabi_Right) / 2.0f;

    fBichang = fBichang * COE_BICHANG;

    return 0;
}

int CMeasure::GetTuichang(float& fTuichang)
{
    float fLenShangtui_left = CalDist(m_stKeyPointsFromNite.stLeftHip, m_stKeyPointsFromNite.stLeftKnee);
    float fLenXiatui_left = CalDist(m_stKeyPointsFromNite.stLeftKnee, m_stKeyPointsFromNite.stLeftFoot);
    float fLenShangtui_Right = CalDist(m_stKeyPointsFromNite.stRightHip, m_stKeyPointsFromNite.stRightKnee);
    float fLenXiatui_Right = CalDist(m_stKeyPointsFromNite.stRightKnee, m_stKeyPointsFromNite.stRightFoot);

    fTuichang = (fLenShangtui_left + fLenXiatui_left + fLenShangtui_Right + fLenXiatui_Right) / 2;

    fTuichang *= 1.0f;

    return 0;
}

int CMeasure::GetXiuquan(float& fXiuquan)
{
    float fXiuquan_Left = CalcDistVertical(m_stKeyPoints.stShoulder_left, m_stKeyPoints.stAxillary_left);
    float fXiuquan_Right = CalcDistVertical(m_stKeyPoints.stShoulder_Right, m_stKeyPoints.stAxillary_Right);

    fXiuquan = (fXiuquan_Left + fXiuquan_Right) / 2;

    fXiuquan = fXiuquan * COE_XIUQUAN_FRONT;

    return 0;
}

int CMeasure::GetShangbiwei(float& fShangbiwei)
{
    PointT pointCalc;
    pointCalc = m_stSkeleton.vecLeftHand->at(0);

    int nIndex = 0;
    float fMinDist = BIG_NUM;
    for(int i = 0 ; i < m_stSkeleton.vecLeftHand->size(); i++)
    {
        float fDist = CalDist(pointCalc, m_stSkeleton.vecLeftHand->at(i));
        if (fDist < fMinDist)
        {
            fMinDist = fDist;
            nIndex = i;
        }
    }

    if (nIndex < 3)
    {
        nIndex = 3;
    }

    PointT pointStart = PointT(0.0f, 0.0f, 0.0f);
    for (int i = nIndex ; i > nIndex - 3 ; i--)
    {
        pointStart = pointStart + m_stSkeleton.vecLeftHand->at(i);
    }
    pointStart = pointStart / 3.0f;

    PointT pointEnd = PointT(0.0f, 0.0f, 0.0f);
    for (int i = nIndex + 1; i < nIndex + 4; i++)
    {
        pointEnd = pointEnd + m_stSkeleton.vecLeftHand->at(i);
    }
    pointEnd = pointEnd / 3.0f;

    PointT vecDirection = pointEnd - pointStart;

    float fModule = 1.0f / sqrt(vecDirection.y * vecDirection.y + vecDirection.x * vecDirection.x);
    PointT vecNormal = PointT(vecDirection.y * fModule, -vecDirection.x * fModule, 0);

    float fDist_Left = CalcDistOblique(pointCalc, vecNormal, OW_EM_LEFT);
    float fDist_Right = CalcDistOblique(pointCalc, vecNormal, OW_EM_RIGHT);
    fShangbiwei = fDist_Left + fDist_Right;

    fShangbiwei = fShangbiwei * COE_SHANGBIWEI;

    return 0;
}

float CMeasure::CalcDistOblique(PointT& stPoint, PointT& stNormal, EMSide emSide)
{
    float fDist = 0;
    PointT stPointNow = stPoint;
    float fPosX_Now = stPointNow.x;
    float fPosY_Now = stPointNow.y;
    int nPosX_Now = (stPointNow.x - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
    int nPosY_Now = (stPointNow.y - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;
    float fPosZ_Now = m_matDepthMap.at<float>(nPosY_Now, nPosX_Now);
    if (fPosZ_Now < SMALL_NUM + 1)
    {
        return -1;
    }

    float fPosX_Next = fPosX_Now;
    float fPosY_Next = fPosY_Now;
    float fPosZ_Next = fPosZ_Now;
    float fDist_Left = 0;

    while (true)
    {
        if (emSide == OW_EM_LEFT)
        {
            fPosX_Next = fPosX_Next - stNormal.x;
            fPosY_Next = fPosY_Next - stNormal.y;
        }
        else
        {
            fPosX_Next = fPosX_Next + stNormal.x;
            fPosY_Next = fPosY_Next + stNormal.y;
        }

        int nPosX_Next = (fPosX_Next - m_stBoundingBox.fLeft) / m_fGridStep + SMALL_FLOAT;
        int nPosY_Next = (fPosY_Next - m_stBoundingBox.fDown) / m_fGridStep + SMALL_FLOAT;

        if (nPosX_Next == nPosX_Now && nPosY_Next == nPosY_Now)
        {
            continue;
        }

        float fPosZ_Next = m_matDepthMap.at<float>(nPosY_Next, nPosX_Next);
        if (fPosZ_Next < SMALL_NUM + 1)
        {
            break;
        }

        fDist_Left = fDist_Left + CalDist(fPosX_Next, fPosY_Next, fPosZ_Next, fPosX_Now, fPosY_Now, fPosZ_Now);

        fPosX_Now = fPosX_Next;
        fPosY_Now = fPosY_Next;
        fPosZ_Now = fPosZ_Next;
        nPosX_Now = nPosX_Next;
        nPosY_Now = nPosY_Next;
    }

    return fDist_Left;
}

int CMeasure::GetLingwei(float& fLingwei)
{
    float fLeft = CalDist(m_stKeyPoints.stNeck_Middle, m_stKeyPoints.stNeck_Left);
    float fRight = CalDist(m_stKeyPoints.stNeck_Middle, m_stKeyPoints.stNeck_Right);
    fLingwei = (fLeft + fRight) * 1.1;

    fLingwei = fLingwei * COE_LINGWEI;

    return 0;
}