#include "Render.h"
#include <gl/GLU.h>
#include <QtCore/QEvent>
#include <QtGui/QResizeEvent>

CRender::CRender(QWidget* parent /* = Q_NULLPTR */)
    :QOpenGLWidget(parent)
{
    m_nWindowWidth              = 0;
    m_nWindowHeight             = 0;
    m_vecRotate.x               = 0;
    m_vecRotate.y               = 0;
    m_vecMousePos_Old.x         = 0;
    m_vecMousePos_Old.y         = 0;
    m_bDragFlag                 = false;
    m_fScalce                   = 0.0f;
    m_pointCloud                = nullptr;
    m_pSkeleton                 = nullptr;
    m_nGridWidth                = 0;
    m_nGridHeight               = 0;
    m_fGridStep                 = 0.0f;
    m_stSkeleton.vecLeftHand    = nullptr;
    m_stSkeleton.vecRightHand   = nullptr;
    m_stSkeleton.vecLeftLeg     = nullptr;
    m_stSkeleton.vecRightLeg    = nullptr;
    m_stSkeleton.vecTosor       = nullptr;
    memset(&m_stBoundingBox, 0, sizeof(m_stBoundingBox));
}

CRender::~CRender()
{

}

void CRender::initializeGL()
{
    float   fAspect = 0.0f;
    GLfloat light_position[] = { 1.0,1.0,1.0,0.0 };
    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

    m_nWindowWidth  = this->geometry().width();
    m_nWindowHeight = this->geometry().height();
    fAspect = ((float)m_nWindowWidth) / ((float)m_nWindowHeight);

    glClearColor(0, 0, 0, 1);
    glViewport(0, 0, m_nWindowWidth, m_nWindowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, fAspect, 0.1f, 10000.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void CRender::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    float fAspect = (float)w / (float)h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, fAspect, 0.1f, 1000.0f);

    m_nWindowHeight = w;
    m_nWindowWidth = h;
}

void CRender::paintGL()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 200 + m_fScalce,
        0, 0, 0,
        0, 1, 0);
    glTranslatef(m_vecTranslate.x, m_vecTranslate.y, 0);
    glRotatef(m_vecRotate.x, 0, 1, 0);
    glRotatef(m_vecRotate.y, 1, 0, 0);

//     if (m_pointCloud != nullptr)
//     {
//         glPointSize(3);
//         glNormal3f(1.0f, 1.0f, 1.0f);
//         glColor3f(1.0f, 1.0f, 1.0f);
//         glBegin(GL_POINTS);
//         for (int i = 0 ; i < m_pointCloud->size() ; i++)
//         {
//             glVertex3f(m_pointCloud->at(i).x,
//                 m_pointCloud->at(i).y,
//                 m_pointCloud->at(i).z);
//         }
//         glEnd();
//     }


    if (m_matDepthMap.size().width != 0 &&
        m_matDepthMap.size().height != 0)
    {
        /************************************************************************/
        /* ªÊ÷∆π«º‹                                                              */
        /************************************************************************/
        if (m_stSkeleton.vecLeftHand != nullptr)
        {
            glPointSize(10);
            glNormal3f(1.0f, 1.0f, 1.0f);
            glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
            glBegin(GL_POINTS);

            for (int i = 0; i < m_stSkeleton.vecLeftHand->size(); i++)
            {
                PointT point = m_stSkeleton.vecLeftHand->at(i);
                glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
                glVertex3f(point.x, point.y, point.z);
            }

            for (int i = 0; i < m_stSkeleton.vecRightHand->size(); i++)
            {
                PointT point = m_stSkeleton.vecRightHand->at(i);
                glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
                glVertex3f(point.x, point.y, point.z);
            }

            for (int i = 0; i < m_stSkeleton.vecTosor->size(); i++)
            {
                PointT point = m_stSkeleton.vecTosor->at(i);
                glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
                glVertex3f(point.x, point.y, point.z);
            }

            for (int i = 0; i < m_stSkeleton.vecLeftLeg->size(); i++)
            {
                PointT point = m_stSkeleton.vecLeftLeg->at(i);
                glColor4f(1.0f, 0.0f, 1.0f, 1.0f);
                glVertex3f(point.x, point.y, point.z);
            }

            for (int i = 0; i < m_stSkeleton.vecRightLeg->size(); i++)
            {
                glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
                PointT point = m_stSkeleton.vecRightLeg->at(i);
                glVertex3f(point.x, point.y, point.z);
            }
//             for (int i = 0; i < m_pSkeleton->size(); i++)
//             {
//                 PointT point = m_pSkeleton->at(i);
//                 glVertex3f(point.x, point.y, point.z);
//             }

            glEnd();
        }

        glNormal3f(1.0f, 1.0f, 1.0f);
        glColor4f(1.0f, 0.3f, 0.3f, 1.0f);
        glBegin(GL_LINES);

        ///< ◊Û“∏œ¬
        glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
        glVertex3f(m_stKeyPoints.stAxillary_left.x, m_stKeyPoints.stAxillary_left.y, m_stKeyPoints.stAxillary_left.z - 20.0f);
        glVertex3f(m_stKeyPoints.stAxillary_left.x, m_stKeyPoints.stAxillary_left.y, m_stKeyPoints.stAxillary_left.z + 20.0f);

        ///< ◊ÛºÁ∞Ú
        glVertex3f(m_stKeyPoints.stShoulder_left.x, m_stKeyPoints.stShoulder_left.y, m_stKeyPoints.stShoulder_left.z - 20.0f);
        glVertex3f(m_stKeyPoints.stShoulder_left.x, m_stKeyPoints.stShoulder_left.y, m_stKeyPoints.stShoulder_left.z + 20.0f);

        ///< ”““∏œ¬
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glVertex3f(m_stKeyPoints.stAxillary_Right.x, m_stKeyPoints.stAxillary_Right.y, m_stKeyPoints.stAxillary_Right.z - 20.0f);
        glVertex3f(m_stKeyPoints.stAxillary_Right.x, m_stKeyPoints.stAxillary_Right.y, m_stKeyPoints.stAxillary_Right.z + 20.0f);

        ///< ”“ºÁ∞Ú
        glVertex3f(m_stKeyPoints.stShoulder_Right.x, m_stKeyPoints.stShoulder_Right.y, m_stKeyPoints.stShoulder_Right.z - 20.0f);
        glVertex3f(m_stKeyPoints.stShoulder_Right.x, m_stKeyPoints.stShoulder_Right.y, m_stKeyPoints.stShoulder_Right.z + 20.0f);

        ///< æ±÷–µ„
        glColor4f(1.0f, 0.3f, 0.3f, 1.0f);
        glVertex3f(m_stKeyPoints.stNeck_Middle.x, m_stKeyPoints.stNeck_Middle.y, m_stKeyPoints.stNeck_Middle.z - 20.0f);
        glVertex3f(m_stKeyPoints.stNeck_Middle.x, m_stKeyPoints.stNeck_Middle.y, m_stKeyPoints.stNeck_Middle.z + 20.0f);

        ///< ◊Ûæ±≤‡µ„
        glVertex3f(m_stKeyPoints.stNeck_Left.x, m_stKeyPoints.stNeck_Left.y, m_stKeyPoints.stNeck_Left.z - 20.0f);
        glVertex3f(m_stKeyPoints.stNeck_Left.x, m_stKeyPoints.stNeck_Left.y, m_stKeyPoints.stNeck_Left.z + 20.0f);

        ///< ”“æ±≤‡µ„
        glVertex3f(m_stKeyPoints.stNeck_Right.x, m_stKeyPoints.stNeck_Right.y, m_stKeyPoints.stNeck_Right.z - 20.0f);
        glVertex3f(m_stKeyPoints.stNeck_Right.x, m_stKeyPoints.stNeck_Right.y, m_stKeyPoints.stNeck_Right.z + 20.0f);

        ///< ◊Û˜≈µ„
        glVertex3f(m_stKeyPoints.stHip_Left.x, m_stKeyPoints.stHip_Left.y, m_stKeyPoints.stHip_Left.z - 20.0f);
        glVertex3f(m_stKeyPoints.stHip_Left.x, m_stKeyPoints.stHip_Left.y, m_stKeyPoints.stHip_Left.z + 20.0f);

        ///< ”“˜≈µ„
        glVertex3f(m_stKeyPoints.stHip_Right.x, m_stKeyPoints.stHip_Right.y, m_stKeyPoints.stHip_Right.z - 20.0f);
        glVertex3f(m_stKeyPoints.stHip_Right.x, m_stKeyPoints.stHip_Right.y, m_stKeyPoints.stHip_Right.z + 20.0f);

        ///< ◊Û—¸µ„
        glVertex3f(m_stKeyPoints.stWaist_Left.x, m_stKeyPoints.stWaist_Left.y, m_stKeyPoints.stWaist_Left.z - 20.0f);
        glVertex3f(m_stKeyPoints.stWaist_Left.x, m_stKeyPoints.stWaist_Left.y, m_stKeyPoints.stWaist_Left.z + 20.0f);

        ///< ”“—¸µ„
        glVertex3f(m_stKeyPoints.stWaist_Right.x, m_stKeyPoints.stWaist_Right.y, m_stKeyPoints.stWaist_Right.z - 20.0f);
        glVertex3f(m_stKeyPoints.stWaist_Right.x, m_stKeyPoints.stWaist_Right.y, m_stKeyPoints.stWaist_Right.z + 20.0f);

        ///< ◊Û–ÿµ„
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glVertex3f(m_stKeyPoints.stChest_Left.x, m_stKeyPoints.stChest_Left.y, m_stKeyPoints.stChest_Left.z - 20.0f);
        glVertex3f(m_stKeyPoints.stChest_Left.x, m_stKeyPoints.stChest_Left.y, m_stKeyPoints.stChest_Left.z + 20.0f);

        ///< ”“–ÿµ„
        glVertex3f(m_stKeyPoints.stChest_Right.x, m_stKeyPoints.stChest_Right.y, m_stKeyPoints.stChest_Right.z - 20.0f);
        glVertex3f(m_stKeyPoints.stChest_Right.x, m_stKeyPoints.stChest_Right.y, m_stKeyPoints.stChest_Right.z + 20.0f);

        glEnd();


        /************************************************************************/
        /* for test                                                             */
        /************************************************************************/
//         glNormal3f(1.0f, 1.0f, 1.0f);
//         glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
//         glPointSize(5);
//         glBegin(GL_POINTS);
//         int nHeightTmp = 110;
//         int nWidthTmp = 145;
//         float fx = m_stBoundingBox.fLeft + nWidthTmp * m_fGridStep;
//         float fy = m_stBoundingBox.fDown + nHeightTmp * m_fGridStep;
//         float fz = m_matDepthMap.at<float>(nHeightTmp, nWidthTmp);
//         glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
//         glVertex3f(fx, fy, fz);
//         glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
//         glVertex3f(fx, fy, 0);
//         glEnd();

        glNormal3f(1.0f, 1.0f, 1.0f);
        glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
        glPointSize(10);

        glBegin(GL_POINTS);

        DrawPoint(m_stKeyPointsFromNite.stHead);
        DrawPoint(m_stKeyPointsFromNite.stLeftElbow);
        DrawPoint(m_stKeyPointsFromNite.stLeftFoot);
        DrawPoint(m_stKeyPointsFromNite.stLeftHand);
        DrawPoint(m_stKeyPointsFromNite.stLeftHip);
        DrawPoint(m_stKeyPointsFromNite.stLeftKnee);
        DrawPoint(m_stKeyPointsFromNite.stLeftShoulder);
        DrawPoint(m_stKeyPointsFromNite.stNeck);
        DrawPoint(m_stKeyPointsFromNite.stRightElbow);
        DrawPoint(m_stKeyPointsFromNite.stRightFoot);
        DrawPoint(m_stKeyPointsFromNite.stRightHand);
        DrawPoint(m_stKeyPointsFromNite.stRightHip);
        DrawPoint(m_stKeyPointsFromNite.stRightShoulder);
        DrawPoint(m_stKeyPointsFromNite.stTorso);

        glEnd();

        /************************************************************************/
        /* ªÊ÷∆…ÌÃÂ                                                              */
        /************************************************************************/
        glNormal3f(1.0f, 1.0f, 1.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
        glBegin(GL_TRIANGLES);

        for (int i = 0; i < m_nGridHeight - 1; i++)
        {
            for (int j = 0; j < m_nGridWidth - 1; j++)
            {
                float fx = m_stBoundingBox.fLeft + j * m_fGridStep;
                float fy = m_stBoundingBox.fDown + i * m_fGridStep;
                float fz = m_matDepthMap.at<float>(i, j);
                
                float fx1 = fx + m_fGridStep;
                float fy1 = fy;
                float fz1 = m_matDepthMap.at<float>(i, j + 1);

                float fx2 = fx + m_fGridStep;
                float fy2 = fy + m_fGridStep;
                float fz2 = m_matDepthMap.at<float>(i + 1, j + 1);

                float fx3 = fx;
                float fy3 = fy + m_fGridStep;
                float fz3= m_matDepthMap.at<float>(i + 1, j);

                if (fz < -20 || fz1 < -20 || fz2 < -20 || fz3 < -20)
                {
                    continue;
                }

                glm::vec3 line1 = glm::vec3(fx1 - fx, fy1 - fy, fz1 - fz);
                glm::vec3 line2 = glm::vec3(fx2 - fx1, fy2 - fy1, fz2 - fz1);
                glm::vec3 normal1 = glm::cross(line1, line2);
                glm::vec3 normal11 = glm::normalize(normal1);

                glNormal3f(normal11.x, normal11.y, normal11.z);
                glVertex3f(fx, fy, fz);
                glVertex3f(fx1, fy1, fz1);
                glVertex3f(fx2, fy2, fz2);

                glm::vec3 line3 = glm::vec3(fx2 - fx, fy2 - fy, fz2 - fz);
                glm::vec3 line4 = glm::vec3(fx3 - fx2, fy3 - fy2, fz3 - fz2);
                glm::vec3 normal2 = glm::cross(line1, line2);
                glm::vec3 normal22 = glm::normalize(normal2);

                glNormal3f(normal22.x, normal22.y, normal22.z);

                glVertex3f(fx, fy, fz);
                glVertex3f(fx2, fy2, fz2);
                glVertex3f(fx3, fy3, fz3);
            }
        }

        glEnd();
    }

    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void CRender::mousePressEvent(QMouseEvent *event)
{
    if (m_bDragFlag == false)
    {
        m_bDragFlag = true;
        m_vecMousePos_Old.x = event->localPos().x();
        m_vecMousePos_Old.y = event->localPos().y();
    }
}

void CRender::mouseReleaseEvent(QMouseEvent *event)
{
    m_bDragFlag = false;
}

void CRender::mouseMoveEvent(QMouseEvent *event)
{
    if (m_bDragFlag = false)
    {
        return;
    }

    if ((event->buttons() & Qt::LeftButton) &&
        !(event->buttons() & Qt::RightButton))
    {
        m_vecRotate.x += event->localPos().x() - m_vecMousePos_Old.x;
        m_vecRotate.y += event->localPos().y() - m_vecMousePos_Old.y;
        m_vecMousePos_Old.x = event->localPos().x();
        m_vecMousePos_Old.y = event->localPos().y();
    }
    else if ((event->buttons() & Qt::LeftButton) &&
        (event->buttons() & Qt::RightButton))
    {
        m_vecTranslate.x += 0.01 * (event->localPos().x() - m_vecMousePos_Old.x);
        m_vecTranslate.y -= 0.01 * (event->localPos().y() - m_vecMousePos_Old.y);
        m_vecMousePos_Old.x = event->localPos().x();
        m_vecMousePos_Old.y = event->localPos().y();
    }
}

void CRender::wheelEvent(QWheelEvent *event)
{
    int nDelta = event->delta();
    if (nDelta > 0)
    {
        m_fScalce += 2;
    }
    else
    {
        m_fScalce -= 2;
    }
}

int CRender::SetPointCloud(pcl::PointCloud<PointT>::Ptr pTmp)
{
    m_pointCloud = pTmp;
    return 0;
}

int CRender::SetBoundingBox(STBoundingBox stBoX)
{
    memcpy(&m_stBoundingBox, &stBoX, sizeof(STBoundingBox));
    return 0;
}

int CRender::SetGridInfo(int nGridWidth, int nGridHeight, float fGridStep)
{
    m_nGridWidth  = nGridWidth;
    m_nGridHeight = nGridHeight;
    m_fGridStep   = fGridStep;

    return 0;
}

int CRender::SetDepthMap(cv::Mat matTmp)
{
    m_matDepthMap = matTmp.clone();

    return 0;
}

int CRender::SetSkeleton(pcl::PointCloud<PointT>::Ptr pTmp)
{
    m_pSkeleton = pTmp;

    return 0;
}

int CRender::SetSkeleton(STSkeleton stSkeleton)
{
    m_stSkeleton = stSkeleton;

    return 0;
}

int CRender::SetKeyPoints(STKeyPoints& stKeyPoints)
{
    m_stKeyPoints = stKeyPoints;
    return 0;
}

int CRender::SetKeyPointsFromNite(STKeypointsFromNite& stKeyPointsFromNite)
{
    m_stKeyPointsFromNite = stKeyPointsFromNite;
    return 0;
}

int CRender::DrawPoint(PointT& point)
{
    glVertex3f(point.x, point.y, point.z);

    return 0;
}