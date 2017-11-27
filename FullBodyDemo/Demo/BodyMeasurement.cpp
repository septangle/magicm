#include "BodyMeasurement.h"
#include "Measure.h"
#include <QtWidgets/QApplication>
#include "Render.h"
#include "MySocket.h"

CBodyMeasurement::CBodyMeasurement(int argc, char** argv)
{
    m_argc = argc;
    m_argv = argv;
}


CBodyMeasurement::~CBodyMeasurement()
{
}

void CBodyMeasurement::StartLoop()
{
    CGestureDetector gestureDetector;
    gestureDetector.Init();

    CMeasure measureKit;

    while (true)
    {
        int nResult = 0;
        vector <cv::Point3f> vecBodyPoints;
        STKeypointsFromNite stKeypointFromNite;
        gestureDetector.StartDetect(vecBodyPoints, stKeypointFromNite);

        STParam stParam;
        measureKit.Init(vecBodyPoints, stKeypointFromNite);
        nResult = measureKit.StartMeasure(stParam);
        if (nResult == -1)
        {
            continue;
        }

        STDataForRender stData;
        measureKit.GetDataForRender(stData);

        char sz[500];
        //int nCount = sprintf(sz, "{\"Éí¸ß\": \"%d\",\"ÁìÎ§\" : \"%d\", \"¼ç¿í\" : \"%d\",\"ÐØ¿í\" : \"%d\",\"ÑüÎ§\" : \"%d\",\"±Û³¤\" : \"%d\",\"ÍÎÎ§\" : \"%d\",\"ÍÈ³¤\" : \"%d\",\"ÐäÈ¦\" : \"%d\",\"ÉÏ±ÛÎ§\" : \"%d\",\"Ç°Ñü½Ú\" : \"%d\"}", 
        int nCount = sprintf(sz, "{\"body\":[{\"\\u8eab\\u9ad8\": \"%d\",\"\\u9886\\u56f4\" : \"%d\", \"\\u80a9\\u5bbd\" : \"%d\",\"\\u80f8\\u5bbd\" : \"%d\",\"\\u8170\\u56f4\" : \"%d\",\"\\u81c2\\u957f\" : \"%d\",\"\\u81c0\\u56f4\" : \"%d\",\"\\u817f\\u957f\" : \"%d\",\"\\u8896\\u5708\" : \"%d\",\"\\u4e0a\\u81c2\\u56f4\" : \"%d\",\"\\u524d\\u8170\\u8282\" : \"%d\"}]}",
            (int)stParam.fShengao, (int)stParam.fLinwei, (int)stParam.fJiankuan, (int)stParam.fXiongkuan, (int)stParam.fYaowei, (int)stParam.fBichang, (int)stParam.fTunwei, (int)stParam.fTuichang, (int)stParam.fXiuquan, (int)stParam.fShangbiwei, (int)stParam.fQianyaojie);

        CMySocket mySocket;
        mySocket.Send(sz, nCount);

        /************************************************************************/
        /* äÖÈ¾                                                                 */
        /************************************************************************/
        QApplication a(m_argc, m_argv);

        CRender render;
        QRect rect = QRect(200, 200, 800, 600);
        render.setGeometry(rect);
        render.show();

        render.SetDepthMap(stData.matDepthMap);
        render.SetBoundingBox(stData.stBoundingBox);
        render.SetGridInfo(stData.nWidth, LEN, stData.fGridStep);
        render.SetSkeleton(stData.stSkeleton);
        render.SetKeyPoints(stData.stKeypoints);
        render.SetKeyPointsFromNite(stData.stKeypointsFromNite);

        a.exec();
    }
}