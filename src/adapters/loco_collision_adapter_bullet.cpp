
#include <adapters/loco_collision_adapter_bullet.h>

namespace loco {
namespace bullet {

    // @todo: Update base-component classes to accept multiple colliders. Currently, I don't see why
    //        we should have a set-local-position method for colliders if they are always one-to-one
    //        (one collider per 1 body).
    // @todo: Test if compound shape can do the job as well as single collision shapes.
    // @todo: Check with other backends (DART and RAISIM) to check the right limits of the abstraction

    TBulletCollisionAdapter::TBulletCollisionAdapter( TCollision* collisionRef )
        : TICollisionAdapter( collisionRef )
    {
        LOCO_CORE_ASSERT( collisionRef, "TBulletCollisionAdapter >>> expected non-null collision-obj reference" );

        m_bulletCollisionShape = nullptr;
        m_bulletRigidBodyRef = nullptr;
        m_bulletWorldRef = nullptr;

        m_size  = m_collisionRef->size();
        m_size0 = m_collisionRef->size();
        m_scale = { 1.0f, 1.0f, 1.0f };

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_collisionRef ) ? m_collisionRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TBulletCollisionAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TBulletCollisionAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    TBulletCollisionAdapter::~TBulletCollisionAdapter()
    {
        m_bulletCollisionShape = nullptr;
        m_bulletRigidBodyRef = nullptr;
        m_bulletWorldRef = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        const std::string name = ( m_collisionRef ) ? m_collisionRef->name() : "undefined";
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TBulletCollisionAdapter {0} @ {1}", name, loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TBulletCollisionAdapter " << name << " @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TBulletCollisionAdapter::Build()
    {
        m_bulletCollisionShape = CreateCollisionShape( m_collisionRef->data() );
        m_bulletCollisionShape->setMargin( 0.001 );

        m_size  = m_collisionRef->size();
        m_size0 = m_collisionRef->size();
        m_scale = { 1.0f, 1.0f, 1.0f };
    }

    void TBulletCollisionAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_bulletCollisionShape, "TBulletCollisionAdapter::Initialize >>> must have \
                          already created a valid bullet collision shape (missing ->Build call?)" );

        LOCO_CORE_TRACE( "Bullet-backend >>> collision-adapter" );
        LOCO_CORE_TRACE( "\tname            : {0}", m_collisionRef->name() );
        LOCO_CORE_TRACE( "\tcol-size        : {0}", ToString( m_collisionRef->size() ) );
        LOCO_CORE_TRACE( "\tcol-shape       : {0}", ToString( m_collisionRef->shape() ) );
        LOCO_CORE_TRACE( "\tcol-group       : {0}", std::to_string( m_collisionRef->collisionGroup() ) );
        LOCO_CORE_TRACE( "\tcol-mask        : {0}", std::to_string( m_collisionRef->collisionMask() ) );
        LOCO_CORE_TRACE( "\tbt-geom-shape   : {0}", bt_shape_enum_to_str( (BroadphaseNativeTypes)m_bulletCollisionShape->getShapeType() ) );
        LOCO_CORE_TRACE( "\tbt-local-scale  : {0}", ToString( vec3_from_bt( m_bulletCollisionShape->getLocalScaling() ) ) );
        LOCO_CORE_TRACE( "\tbt-margin       : {0}", m_bulletCollisionShape->getMargin() );
        btVector3 aabb_min, aabb_max;
        m_bulletCollisionShape->getAabb( btTransform::getIdentity(), aabb_min, aabb_max );
        btVector3 bsphere_center; btScalar bsphere_radius;
        m_bulletCollisionShape->getBoundingSphere( bsphere_center, bsphere_radius );
        LOCO_CORE_TRACE( "\tbt-aabb (dxyz)  : ({0},{1},{2})", aabb_max.x() - aabb_min.x(), aabb_max.y() - aabb_min.y(), aabb_max.z() - aabb_min.z() );
        LOCO_CORE_TRACE( "\tbt-bsphere-rad  : {0}", bsphere_radius );
        const int bt_shape_type = m_bulletCollisionShape->getShapeType();
        if ( bt_shape_type == COMPOUND_SHAPE_PROXYTYPE )
        {
            auto bt_compound_shape = dynamic_cast<btCompoundShape*>( m_bulletCollisionShape.get() );
            const int bt_num_children = bt_compound_shape->getNumChildShapes();
            LOCO_CORE_TRACE( "\tbt-num-children : {0}", bt_num_children );
            for ( size_t i = 0; i < bt_num_children; i++ )
            {
                LOCO_CORE_TRACE( "\tbt-child-tf     : \n{0}", ToString( mat4_from_bt( bt_compound_shape->getChildTransform( i ) ) ) );
                if ( auto bt_child_shape = bt_compound_shape->getChildShape( i ) )
                {
                    LOCO_CORE_TRACE( "\tbt-child-type   : {0}", bt_shape_enum_to_str( (BroadphaseNativeTypes)bt_child_shape->getShapeType() ) );
                    LOCO_CORE_TRACE( "\tbt-child-loc.scl: {0}", ToString( vec3_from_bt( bt_child_shape->getLocalScaling() ) ) );
                    LOCO_CORE_TRACE( "\tbt-child-margin : {0}", bt_child_shape->getMargin() );
                }

            }
        }
    }

    void TBulletCollisionAdapter::PreStep()
    {
        // Nothing to prepare before to a simulation step
    }

    void TBulletCollisionAdapter::PostStep()
    {
        // Nothing to process after to a simulation step
    }

    void TBulletCollisionAdapter::Reset()
    {
        // Nothing to reset in the backend
    }

    void TBulletCollisionAdapter::SetLocalPosition( const TVec3& position )
    {
        // @todo: Check if will be required (if so, will use btCompoundShape always)
    }

    void TBulletCollisionAdapter::SetLocalRotation( const TMat3& rotation )
    {
        // @todo: Check if will be required (if so, will use btCompoundShape always)
    }

    void TBulletCollisionAdapter::SetLocalTransform( const TMat4& transform )
    {
        // @todo: Check if will be required (if so, will use btCompoundShape always)
    }

    void TBulletCollisionAdapter::ChangeSize( const TVec3& newSize )
    {
        // adapted from gazebo set-size functionality, similar to our drawable's resize using scaling: 
        //      url> https://bitbucket.org/osrf/gazebo/src/804410860234af97c5e309896dc007e8cde04ba8/gazebo/physics/bullet/BulletBoxShape.hh#lines-88
        //      comments> I'm not sure if this will work for capsules. For our drawables, when using scaling
        //                we ran into some issues as scaling deformed the capsule. Will have to check (@todo) if
        //                for all collision shapes scaling just increases base dimensions and everything else is
        //                computed accordingly (collision detection, etc.)

        if ( !m_bulletCollisionShape )
            return;

        m_size = newSize;
        const eShapeType shape = m_collisionRef->shape();
        switch ( shape )
        {
            case eShapeType::PLANE :
            {
                m_scale.x() = m_size.x() / m_size0.x();
                m_scale.y() = m_size.y() / m_size0.y();
                m_scale.z() = 1.0f;
                break;
            }
            case eShapeType::BOX :
            {
                m_scale.x() = m_size.x() / m_size0.x();
                m_scale.y() = m_size.y() / m_size0.y();
                m_scale.z() = m_size.z() / m_size0.z();
                break;
            }
            case eShapeType::SPHERE :
            {
                const auto scale = m_size.x() / m_size0.x();
                m_scale.x() = scale;
                m_scale.y() = scale;
                m_scale.z() = scale;
                break;
            }
            case eShapeType::CYLINDER :
            {
                const auto scale_radius = m_size.x() / m_size0.x();
                const auto scale_height = m_size.y() / m_size0.y();
                m_scale.x() = scale_radius;
                m_scale.y() = scale_radius;
                m_scale.z() = scale_height;
                break;
            }
            case eShapeType::CAPSULE :
            {
                const auto scale_radius = m_size.x() / m_size0.x();
                const auto scale_height = m_size.y() / m_size0.y();
                m_scale.x() = scale_radius;
                m_scale.y() = scale_radius;
                m_scale.z() = scale_height;
                break;
            }
            case eShapeType::MESH :
            {
                m_scale.x() = m_size.x() / m_size0.x();
                m_scale.y() = m_size.y() / m_size0.y();
                m_scale.z() = m_size.z() / m_size0.z();
                break;
            }
            case eShapeType::HFIELD :
            {
                m_scale.x() = m_size.x() / m_size0.x();
                m_scale.y() = m_size.y() / m_size0.y();
                m_scale.z() = m_size.z() / m_size0.z();
                break;
            }
        }

        m_bulletCollisionShape->setLocalScaling( vec3_to_bt( m_scale ) );
    }

    void TBulletCollisionAdapter::ChangeElevationData( const std::vector<float>& heights )
    {
        // Nothing extra here. Adaptee already updates heights buffer using memcpy to keep the
        // pointer to heights-data for bullet (recall we have to maintain that pointer valid and alive)

        // @todo: Check if scale-z does the trick to update the max-height. If not, then would have
        //        to create a fork of bullet and make the required functionality for that case
    }

    void TBulletCollisionAdapter::ChangeCollisionGroup( int collisionGroup )
    {
        if ( !m_bulletRigidBodyRef )
        {
            LOCO_CORE_ERROR( "TBulletCollisionAdapter::ChangeCollisionGroup >>> collider {0} doesn't \
                              have a handle to its associated bullet rigid-body owner", m_collisionRef->name() );
            return;
        }

        m_bulletRigidBodyRef->getBroadphaseHandle()->m_collisionFilterGroup = collisionGroup;
    }

    void TBulletCollisionAdapter::ChangeCollisionMask( int collisionMask )
    {
        if ( !m_bulletRigidBodyRef )
        {
            LOCO_CORE_ERROR( "TBulletCollisionAdapter::ChangeCollisionMask >>> collider {0} doesn't \
                              have a handle to its associated bullet rigid-body owner", m_collisionRef->name() );
            return;
        }

        m_bulletRigidBodyRef->getBroadphaseHandle()->m_collisionFilterMask = collisionMask;
    }

}}