#pragma once

#include "GlobalHeader.h"
#include <QtWidgets/QOpenGLWidget>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class CRender : public QOpenGLWidget
{
public:
    CRender(QWidget* parent = Q_NULLPTR);
    ~CRender();
    int SetPointCloud(pcl::PointCloud<PointT>::Ptr pTmp);
    int SetDepthMap(cv::Mat matTmp);
    int SetBoundingBox(STBoundingBox stBoX);
    int SetGridInfo(int nGridWidth, int nGridHeight, float fGridStep);
    int SetSkeleton(pcl::PointCloud<PointT>::Ptr pTmp);
    int SetSkeleton(STSkeleton stSkeleton);
    int SetKeyPoints(STKeyPoints& stKeyPoints);
    int SetKeyPointsFromNite(STKeypointsFromNite& stKeyPointsFromNite);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    int DrawPoint(PointT& point);

private:
    int             m_nWindowWidth;
    int             m_nWindowHeight;
    glm::vec2       m_vecRotate;
    glm::vec2       m_vecTranslate;
    float           m_fScalce;
    glm::vec2       m_vecMousePos_Old;
    bool            m_bDragFlag;
    pcl::PointCloud<PointT>::Ptr m_pointCloud;
    pcl::PointCloud<PointT>::Ptr m_pSkeleton;
    STBoundingBox   m_stBoundingBox;
    int             m_nGridWidth;
    int             m_nGridHeight;
    float           m_fGridStep;
    cv::Mat         m_matDepthMap;
    STSkeleton      m_stSkeleton;
    STKeyPoints     m_stKeyPoints;
    STKeypointsFromNite     m_stKeyPointsFromNite;
};

