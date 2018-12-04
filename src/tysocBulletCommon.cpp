
#include <tysocBulletCommon.h>



namespace tysocBullet
{

    void createBtVec3( float* srcVec, btVector3& outVec )
    {
        outVec.setX( srcVec[0] );
        outVec.setY( srcVec[1] );
        outVec.setZ( srcVec[2] );
    }

    void getVec3Array( const btVector3& srcVec, float* outVec )
    {
        outVec[0] = srcVec.x();
        outVec[1] = srcVec.y();
        outVec[2] = srcVec.z();
    }

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

    void getMat3Array( const btMatrix3x3& srcMat, float* outMat )
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