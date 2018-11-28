
#include <tysocBulletCommon.h>



namespace tysocBullet
{


    void createBtMat3( float* srcMat, btMatrix3x3& outMat )
    {
        for ( size_t i = 0; i < 3; i++ )
        {
            for ( size_t j = 0; j < 3; j++ )
            {
                outMat[i][j] = srcMat[i + 3 * j];
            }
        }
    }

    void getBtMat3( const btMatrix3x3& srcMat, float* outMat )
    {
        for ( size_t i = 0; i < 3; i++ )
        {
            for ( size_t j = 0; j < 3; j++ )
            {
                outMat[i + 3 * j] = srcMat[i][j];
            }
        }
    }
}