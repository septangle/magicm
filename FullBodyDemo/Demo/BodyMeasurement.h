#pragma once

#include "GestureDetector.h"

class CBodyMeasurement
{
public:
    CBodyMeasurement(int argc, char** argv);
    ~CBodyMeasurement();
    void StartLoop();

private:
    int         m_argc;                                                           ///< main函数传入参数
    char**      m_argv;                                                           ///< main函数传入参数
};

