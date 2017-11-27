#include "Math.h"
#include "GlobalHeader.h"

int CMath::GaussianBlur(cv::Mat& inputMat)
{
    cv::Mat gaussMat = (cv::Mat_<uint16_t>(5, 5) <<
        1, 1, 1, 1, 1,
        1, 3, 5, 3, 1,
        1, 5, 10, 5, 1,
        1, 3, 5, 3, 1,
        1, 1, 1, 1, 1);

    cv::Mat matClone = inputMat.clone();

    for (int i = 0; i < matClone.size().height; i++)
    {
        for (int j = 0; j < matClone.size().width; j++)
        {
            if (i - 2 < 0 || i + 2 > matClone.size().height - 1 ||
                j - 2 < 0 || j + 2 > matClone.size().width - 1)
            {
                continue;
            }

            int nDepth = matClone.ptr<uint16_t>(i)[j];
            if (nDepth == MAX_DEPTH)
            {
                continue;
            }

            int nCount = 0;
            int nResult = 0;
            for (int k = i - 2; k <= i + 2; k++)
            {
                for (int l = j - 2; l <= j + 2; l++)
                {
                    uint16_t nDepth = matClone.ptr<uint16_t>(k)[l];
                    if (nDepth == MAX_DEPTH)
                    {
                        continue;
                    }

                    nCount += gaussMat.ptr<uint16_t>(k - i + 2)[l - j + 2];
                    nResult += (nDepth * gaussMat.ptr<uint16_t>(k - i + 2)[l - j + 2]);
                }
            }

            if (nCount == 0)
            {
                inputMat.ptr<uint16_t>(i)[j] = MAX_DEPTH;
            }
            else
            {
                inputMat.ptr<uint16_t>(i)[j] = nResult / nCount;
            }

        }
    }

    return 0;
}

uint16_t CMath::GetAverage(uint16_t szDepth[AVERAGE_TIMES])
{
    uint16_t nResult = 0;

    for (int i = 0; i < AVERAGE_TIMES; i++)
    {
        for (int j = i; j < AVERAGE_TIMES; j++)
        {
            if (szDepth[i] > szDepth[j])
            {
                int temp = szDepth[i];
                szDepth[i] = szDepth[j];
                szDepth[j] = temp;
            }
        }
    }

    nResult = szDepth[4];

    return nResult;
}