#include "GestureDetector.h"
#include "Math.h"

CGestureDetector::CGestureDetector()
{
}


CGestureDetector::~CGestureDetector()
{
    // 销毁彩色数据流和深度数据流
    m_colorStream.destroy();
    m_depthStream.destroy();

    // 关闭Kinect设备
    m_device.close();

    // 关闭NITE和OpenNI环境
    NiTE::shutdown();
    OpenNI::shutdown();
}

int CGestureDetector::Init()
{
    // 初始化OpenNI
    OpenNI::initialize();

    // 打开设备
    m_device.open(ANY_DEVICE);

    // 创建深度数据流
    m_depthStream.create(m_device, SENSOR_DEPTH);

    // 设置VideoMode模式
    m_depthMode.setResolution(640, 480);
    m_depthMode.setFps(30);
    m_depthMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    m_depthStream.setVideoMode(m_depthMode);

    // 同样的设置彩色数据流
    m_colorStream.create(m_device, SENSOR_COLOR);
    // 设置VideoMode模式
    VideoMode mColorMode;
    mColorMode.setResolution(640, 480);
    mColorMode.setFps(30);
    mColorMode.setPixelFormat(PIXEL_FORMAT_RGB888);
    m_colorStream.setVideoMode(mColorMode);

    // 设置深度图像映射到彩色图像
    m_device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    // 为了得到骨骼数据，先初始化NiTE
    NiTE::initialize();

    // 创建User彩色图像显示
    cv::namedWindow("User Image", CV_WINDOW_AUTOSIZE);

    // 环境初始化后，开始获取深度数据流和彩色数据流
    m_depthStream.start();
    m_colorStream.start();

    return 0;
}

PointT CGestureDetector::convert(const nite::Point3f& in)
{
    PointT point;
    point.x = in.x * ZOOM_SCALE;
    point.y = in.y * ZOOM_SCALE;
    point.z = in.z * (ZOOM_SCALE * (-1));

    return point;
}

int CGestureDetector::StartDetect(std::vector<cv::Point3f>& vecBodyPoints, STKeypointsFromNite& stKeyPoints)
{
    // 创建用户跟踪器
    UserTracker usertracker;
    usertracker.create(&m_device);

    // Control the smoothing factor of the skeleton joints. Factor should be between 0 (no smoothing at all) and 1 (no movement at all)
    usertracker.setSkeletonSmoothingFactor(0.1f);

    while (true)
    {
        // 创建OpenCV：：Mat，用于显示彩色数据图像
        cv::Mat cImageBGR;

        // 读取彩色图像数据帧信息流
        VideoFrameRef mColorFrame;
        m_colorStream.readFrame(&mColorFrame);

        // 将彩色数据流转换为OpenCV格式，记得格式是：CV_8UC3（含R\G\B）
        const cv::Mat mImageRGB(mColorFrame.getHeight(), mColorFrame.getWidth(),
            CV_8UC3, (void*)mColorFrame.getData());

        // RGB ==> BGR
        cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);

        // 读取User用户数据帧信息流
        UserTrackerFrameRef  mUserFrame;
        usertracker.readFrame(&mUserFrame);

        // 得到Users信息
        const nite::Array<UserData>& aUsers = mUserFrame.getUsers();

        if (aUsers.getSize() == 1)
        {
            const UserData& rUser = aUsers[0];

            // 检查用户状态
            if (rUser.isNew())
            {
                // 开始对该用户的骨骼跟踪
                usertracker.startSkeletonTracking(rUser.getId());
            }

            if (rUser.isVisible())
            {
                // 得到用户骨骼数据
                const Skeleton& rSkeleton = rUser.getSkeleton();

                // 检查骨骼状态是否为“跟踪状态”
                if (rSkeleton.getState() == SKELETON_TRACKED)
                {
                    // 得到15个骨骼数据
                    SkeletonJoint aJoints[15];
                    aJoints[0] = rSkeleton.getJoint(JOINT_HEAD);
                    aJoints[1] = rSkeleton.getJoint(JOINT_NECK);
                    aJoints[2] = rSkeleton.getJoint(JOINT_LEFT_SHOULDER);
                    aJoints[3] = rSkeleton.getJoint(JOINT_RIGHT_SHOULDER);
                    aJoints[4] = rSkeleton.getJoint(JOINT_LEFT_ELBOW);
                    aJoints[5] = rSkeleton.getJoint(JOINT_RIGHT_ELBOW);
                    aJoints[6] = rSkeleton.getJoint(JOINT_LEFT_HAND);
                    aJoints[7] = rSkeleton.getJoint(JOINT_RIGHT_HAND);
                    aJoints[8] = rSkeleton.getJoint(JOINT_TORSO);
                    aJoints[9] = rSkeleton.getJoint(JOINT_LEFT_HIP);
                    aJoints[10] = rSkeleton.getJoint(JOINT_RIGHT_HIP);
                    aJoints[11] = rSkeleton.getJoint(JOINT_LEFT_KNEE);
                    aJoints[12] = rSkeleton.getJoint(JOINT_RIGHT_KNEE);
                    aJoints[13] = rSkeleton.getJoint(JOINT_LEFT_FOOT);
                    aJoints[14] = rSkeleton.getJoint(JOINT_RIGHT_FOOT);

                    // 将骨骼3D坐标转换为深度坐标下骨骼位置坐标，并保存在数组中
                    cv::Point2f aPoint[15];
                    for (int s = 0; s < 15; ++s)
                    {
                        const nite::Point3f& rPos = aJoints[s].getPosition();
                        usertracker.convertJointCoordinatesToDepth(
                            rPos.x, rPos.y, rPos.z,
                            &(aPoint[s].x), &(aPoint[s].y));
                    }

                    // 在彩色图像中画出骨骼间的连接线
                    cv::line(cImageBGR, aPoint[0], aPoint[1], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[1], aPoint[2], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[1], aPoint[3], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[2], aPoint[4], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[3], aPoint[5], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[4], aPoint[6], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[5], aPoint[7], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[1], aPoint[8], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[8], aPoint[9], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[8], aPoint[10], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[9], aPoint[11], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[10], aPoint[12], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[11], aPoint[13], cv::Scalar(255, 0, 0), 3);
                    cv::line(cImageBGR, aPoint[12], aPoint[14], cv::Scalar(255, 0, 0), 3);

                    // 同样的在彩色图像中骨骼位置上画“圆”
                    for (int s = 0; s < 15; ++s)
                    {
                        if (aJoints[s].getPositionConfidence() > 0.5)
                            cv::circle(cImageBGR, aPoint[s], 3, cv::Scalar(0, 0, 255), 2);
                        else
                            cv::circle(cImageBGR, aPoint[s], 3, cv::Scalar(0, 255, 0), 2);
                    }

                    if (aPoint[2].x > aPoint[4].x + 30 && aPoint[2].x > aPoint[6].x + 30 &&
                        aPoint[3].x < aPoint[5].x - 30 && aPoint[3].x < aPoint[7].x - 30 &&
                        aUsers.getSize() == 1 &&
                        aPoint[2].y < aPoint[4].y - 20 && aPoint[4].y < aPoint[6].y - 20 &&
                        aPoint[3].y < aPoint[5].y - 20 && aPoint[5].y < aPoint[7].y - 20
                        )
                    {
                        openni::VideoFrameRef szDepthFrameTemp[AVERAGE_TIMES];

                        for (int nFrameIndex = 0; nFrameIndex < AVERAGE_TIMES; nFrameIndex++)
                        {
                            m_depthStream.readFrame(&szDepthFrameTemp[nFrameIndex]);
                            Sleep(100);
                        }

                        int nWidth = szDepthFrameTemp[0].getWidth();
                        int nHeight = szDepthFrameTemp[0].getHeight();

                        cv::Mat matDepth(nHeight, nWidth, CV_16UC1);

                        for (int i = 0; i < nHeight; i++)
                        {
                            for (int j = 0; j < nWidth; j++)
                            {
                                uint16_t szDepth[AVERAGE_TIMES];
                                for (int k = 0; k < AVERAGE_TIMES; k++)
                                {
                                    uint16_t* pDepth = (uint16_t*)szDepthFrameTemp[k].getData();
                                    szDepth[k] = *(pDepth + i * nWidth + j);
                                }

                                uint16_t nAverage = CMath::GetAverage(szDepth);

                                if (nAverage > MAX_DEPTH || nAverage < MIN_DEPTH)
                                {
                                    matDepth.ptr<uint16_t>(i)[j] = MAX_DEPTH;
                                }
                                else
                                {
                                    matDepth.ptr<uint16_t>(i)[j] = nAverage;
                                }
                            }
                        }

                        stKeyPoints.stHead = convert(aJoints[0].getPosition());
                        stKeyPoints.stNeck = convert(aJoints[1].getPosition());
                        stKeyPoints.stLeftShoulder = convert(aJoints[2].getPosition());
                        stKeyPoints.stRightShoulder = convert(aJoints[3].getPosition());
                        stKeyPoints.stLeftElbow = convert(aJoints[4].getPosition());
                        stKeyPoints.stRightElbow = convert(aJoints[5].getPosition());
                        stKeyPoints.stLeftHand = convert(aJoints[6].getPosition());
                        stKeyPoints.stRightHand = convert(aJoints[7].getPosition());
                        stKeyPoints.stTorso = convert(aJoints[8].getPosition());
                        stKeyPoints.stLeftHip = convert(aJoints[9].getPosition());
                        stKeyPoints.stRightHip = convert(aJoints[10].getPosition());
                        stKeyPoints.stLeftKnee = convert(aJoints[11].getPosition());
                        stKeyPoints.stRightKnee = convert(aJoints[12].getPosition());
                        stKeyPoints.stLeftFoot = convert(aJoints[13].getPosition());
                        stKeyPoints.stRightFoot = convert(aJoints[14].getPosition());
//                         stKeyPoints.stHead = convert(matDepth, m_depthStream, aPoint[0]);
//                         stKeyPoints.stNeck = convert(matDepth, m_depthStream, aPoint[1]);
//                         stKeyPoints.stLeftShoulder = convert(matDepth, m_depthStream, aPoint[2]);
//                         stKeyPoints.stRightShoulder = convert(matDepth, m_depthStream, aPoint[3]);
//                         stKeyPoints.stLeftElbow = convert(matDepth, m_depthStream, aPoint[4]);
//                         stKeyPoints.stRightElbow = convert(matDepth, m_depthStream, aPoint[5]);
//                         stKeyPoints.stLeftHand = convert(matDepth, m_depthStream, aPoint[6]);
//                         stKeyPoints.stRightHand = convert(matDepth, m_depthStream, aPoint[7]);
//                         stKeyPoints.stTorso = convert(matDepth, m_depthStream, aPoint[8]);
//                         stKeyPoints.stLeftHip = convert(matDepth, m_depthStream, aPoint[9]);
//                         stKeyPoints.stRightHip = convert(matDepth, m_depthStream, aPoint[10]);
//                         stKeyPoints.stLeftKnee = convert(matDepth, m_depthStream, aPoint[11]);
//                         stKeyPoints.stRightKnee = convert(matDepth, m_depthStream, aPoint[12]);
//                         stKeyPoints.stLeftFoot = convert(matDepth, m_depthStream, aPoint[13]);
//                         stKeyPoints.stRightFoot = convert(matDepth, m_depthStream, aPoint[14]);

                        nite::BoundingBox box = rUser.getBoundingBox();

                        if (box.min.x - 10 > 0)
                        {
                            box.min.x = box.min.x - 10;
                        }

                        if (box.max.x + 10 < nWidth)
                        {
                            box.max.x = box.max.x + 10;
                        }

                        if (box.min.y - 10 > 0)
                        {
                            box.min.y = box.min.y - 10;
                        }

                        if (box.max.y + 10 < nHeight)
                        {
                            box.max.y = box.max.y + 10;
                        }
                        //box.max.y = matDepth.size().height;

                        cv::Mat bodyDepthOrigin = matDepth(cv::Range(box.min.y, box.max.y), cv::Range(box.min.x, box.max.x));

                        cv::imshow("111", bodyDepthOrigin);

                        int nBodyDepthOriginWidth = bodyDepthOrigin.size().width;
                        int nBodyDepthOriginHeight = bodyDepthOrigin.size().height;
                        uint16_t nCenterDepth = bodyDepthOrigin.ptr<uint16_t>(nBodyDepthOriginHeight / 2)[nBodyDepthOriginWidth / 2];
                        for (int i = 0; i < nBodyDepthOriginHeight; i++)
                        {
                            for (int j = 0; j < nBodyDepthOriginWidth; j++)
                            {
                                int nDepth = bodyDepthOrigin.ptr<uint16_t>(i)[j];
                                if (nDepth - nCenterDepth > 300)
                                {
                                    bodyDepthOrigin.ptr<uint16_t>(i)[j] = MAX_DEPTH;
                                }
                            }
                        }

                        CMath::GaussianBlur(bodyDepthOrigin);

                        openni::CoordinateConverter convertor;

                        float fMinHeight = 100000;
                        vector<cv::Point3f> vecTemp;
                        for (int i = 0; i < nBodyDepthOriginHeight; i++)
                        {
                            for (int j = 0; j < nBodyDepthOriginWidth; j++)
                            {
                                cv::Point3f worldPos;
                                int x = j + box.min.x;
                                int y = i + box.min.y;
                                uint16_t depth = bodyDepthOrigin.ptr<uint16_t>(i)[j];
                                if (depth == MAX_DEPTH)
                                {
                                    continue;
                                }
                                convertor.convertDepthToWorld(m_depthStream, x, y, depth, &worldPos.x, &worldPos.y, &worldPos.z);
                                vecTemp.push_back(worldPos);
                                if (worldPos.y < fMinHeight)
                                {
                                    fMinHeight = worldPos.y;
                                }
                            }
                        }

                        for (int i = 0; i < vecTemp.size(); i++)
                        {
                            if (vecTemp.at(i).y > fMinHeight + 30)
                            {
                                cv::Point3f point;
                                point.x = vecTemp.at(i).x * ZOOM_SCALE;
                                point.y = vecTemp.at(i).y * ZOOM_SCALE;
                                point.z = vecTemp.at(i).z * (ZOOM_SCALE * (-1));

                                vecBodyPoints.push_back(point);
                            }
                        }

                        break;
                    }
                }
            }
        }



        cv::imshow("Detector", cImageBGR);
        cv::waitKey(10);
    }

    return 0;
}

PointT CGestureDetector::convert(cv::Mat mat, VideoStream& mDepthStream, cv::Point2f point2f)
{
    openni::CoordinateConverter convertor;

    uint16_t depth = mat.ptr<uint16_t>((int)point2f.y)[(int)point2f.x];
    PointT point;
    convertor.convertDepthToWorld(mDepthStream, (int)point2f.x, (int)point2f.y, depth, &point.x, &point.y, &point.z);
    point.x *= ZOOM_SCALE;
    point.y *= ZOOM_SCALE;
    point.z *= (ZOOM_SCALE * (-1));
    return point;
}