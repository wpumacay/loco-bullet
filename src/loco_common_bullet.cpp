
#include <loco_common_bullet.h>

namespace loco {
namespace bullet {

    btVector3 vec3_to_bt( const TVec3& vec )
    {
        return btVector3( vec.x(), vec.y(), vec.z() );
    }

    btMatrix3x3 mat3_to_bt( const TMat3& mat )
    {
        btMatrix3x3 bt_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                bt_mat[i][j] = mat( i, j );
        return bt_mat;
    }

    btTransform mat4_to_bt( const TMat4& mat )
    {
        return btTransform( mat3_to_bt( TMat3( mat ) ),
                            vec3_to_bt( TVec3( mat.col( 3 ) ) ) );
    }

    TVec3 vec3_from_bt( const btVector3& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    TMat3 mat3_from_bt( const btMatrix3x3& mat )
    {
        TMat3 tm_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                tm_mat( i, j ) = mat[i][j];
        return tm_mat;
    }

    TMat4 mat4_from_bt( const btTransform& mat )
    {
        return TMat4( mat3_from_bt( mat.getBasis() ),
                      vec3_from_bt( mat.getOrigin() ) );
    }

}}