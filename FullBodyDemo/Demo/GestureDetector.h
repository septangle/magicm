#pragma once
#include "GlobalHeader.h"
#include <OpenNI.h>
#include <NiTE.h>

using namespace std;
using namespace openni;
using namespace nite;

class CGestureDetector
{
public:
    CGestureDetector();
    ~CGestureDetector();
    int Init();
    int StartDetect(std::vector<cv::Point3f>& vecBodyPoints, STKeypointsFromNite& stKeyPoints);

private:
    PointT convert(cv::Mat mat, VideoStream& mDepthStream, cv::Point2f point2f);
    PointT convert(const nite::Point3f& in);

private:
    Device  m_device;
    VideoStream m_depthStream;
    VideoMode m_depthMode;
    VideoStream m_colorStream;
};

