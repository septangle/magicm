#include "GlobalHeader.h"
#include "BodyMeasurement.h"

int main(int argc, char** argv)
{
    CBodyMeasurement bodyMeasurement(argc, argv);
    bodyMeasurement.StartLoop();
    return 0;
}