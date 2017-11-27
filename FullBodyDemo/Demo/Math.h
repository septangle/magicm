#pragma once
#include "GlobalHeader.h"

class CMath
{
public:
    static int GaussianBlur(cv::Mat& inputMat);
    static uint16_t GetAverage(uint16_t szDepth[AVERAGE_TIMES]);
};

