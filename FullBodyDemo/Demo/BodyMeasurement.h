#pragma once

#include "GestureDetector.h"

class CBodyMeasurement
{
public:
    CBodyMeasurement(int argc, char** argv);
    ~CBodyMeasurement();
    void StartLoop();

private:
    int         m_argc;                                                           ///< main�����������
    char**      m_argv;                                                           ///< main�����������
};

