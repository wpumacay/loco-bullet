
#include <adapters/bullet_collision_adapter.h>

using namespace tysoc::bullet;

namespace tysoc
{

    TBtCollisionAdapter::TBtCollisionAdapter( TCollision* collision )
        : TICollisionAdapter( collision )
    {
        m_btCollisionShape = nullptr;
        m_btCompoundShapeRef = nullptr;
        m_indexInCompoundShape = -1;
        m_btCollisionLocalTf.setIdentity();
    }

    TBtCollisionAdapter::~TBtCollisionAdapter()
    {
        m_btCollisionShape = nullptr;
        m_btCompoundShapeRef = nullptr;
        m_indexInCompoundShape = -1;
    }

    void TBtCollisionAdapter::build()
    {
        // create collider according to user-data
        m_btCollisionShape = std::unique_ptr< btCollisionShape >( utils::createCollisionShape( m_collisionPtr->data() ) );
        // save initial size for resizing purposes
        m_size0 = m_size = m_collisionPtr->data().size;
        // set this as base scale for resizing
        m_scale = { 1.0f, 1.0f, 1.0f };
        // keep shape type
        m_type = m_collisionPtr->data().type;
    }

    void TBtCollisionAdapter::reset()
    {
        // nothing required for now, as the functionality exposed in the other methods seems enough
    }

    void TBtCollisionAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TBtCollisionAdapter::setLocalPosition( const TVec3& position )
    {
        if ( !m_btCompoundShapeRef || m_indexInCompoundShape == -1 )
            return; // @todo: add logs

        // @todo: should recompute inertia matrix in body-adapter, as the collider defines the mass + inertia
        //        of a component of the body (in case custom inertia-props were not given).

        btTransform _newTransform( m_btCompoundShapeRef->getChildTransform( m_indexInCompoundShape ) );
        _newTransform.setOrigin( utils::toBtVec3( position ) );

        m_btCompoundShapeRef->updateChildTransform( m_indexInCompoundShape, _newTransform, true );
    }

    void TBtCollisionAdapter::setLocalRotation( const TMat3& rotation )
    {
        if ( !m_btCompoundShapeRef || m_indexInCompoundShape == -1 )
            return; // @todo: add logs

        // @todo: should recompute inertia matrix in body-adapter, as the collider defines the mass + inertia
        //        of a component of the body (in case custom inertia-props were not given).

        btTransform _newTransform( m_btCompoundShapeRef->getChildTransform( m_indexInCompoundShape ) );
        _newTransform.setBasis( utils::toBtMat3( rotation ) );

        m_btCompoundShapeRef->updateChildTransform( m_indexInCompoundShape, _newTransform, true );
    }

    void TBtCollisionAdapter::setLocalTransform( const TMat4& transform )
    {
        if ( !m_btCompoundShapeRef || m_indexInCompoundShape == -1 )
            return; // @todo: add logs

        // @todo: should recompute inertia matrix in body-adapter, as the collider defines the mass + inertia
        //        of a component of the body (in case custom inertia-props were not given).

        btTransform _newTransform = utils::toBtTransform( transform );

        m_btCompoundShapeRef->updateChildTransform( m_indexInCompoundShape, _newTransform, true );
    }

    void TBtCollisionAdapter::changeSize( const TVec3& newSize )
    {
        // adapted from gazebo set-size functionality, similar to our drawable's resize using scaling: 
        //      url> https://bitbucket.org/osrf/gazebo/src/804410860234af97c5e309896dc007e8cde04ba8/gazebo/physics/bullet/BulletBoxShape.hh#lines-88
        //      comments> I'm not sure if this will work for capsules. For our drawables, when using scaling
        //                we ran into some issues as scaling deformed the capsule. Will have to check (@todo) if
        //                for all collision shapes scaling just increases base dimensions and everything else is
        //                computed accordingly (collision detection, etc.)

        if ( !m_btCollisionShape )
            return; // @todo: add logs

        m_size = newSize;
        if ( m_type == eShapeType::PLANE )
        {
            m_scale.x = m_size.x / m_size0.x;
            m_scale.y = m_size.y / m_size0.y;
            m_scale.z = 1.0f;
        }
        else if ( m_type == eShapeType::BOX )
        {
            m_scale = { m_size.x / m_size0.x,
                        m_size.y / m_size0.y,
                        m_size.z / m_size0.z };
        }
        else if ( m_type == eShapeType::SPHERE )
        {
            // scale according to radius
            m_scale.x = m_scale.y = m_scale.z = m_size.x / m_size0.x;
        }
        else if ( m_type == eShapeType::CYLINDER )
        {
            // scale according to radius
            m_scale.x = m_size.x / m_size0.x;
            m_scale.y = m_size.x / m_size0.x;
            // scale according to height
            m_scale.z = m_size.y / m_size0.y;
        }
        else if ( m_type == eShapeType::CAPSULE )
        {
            // scale according to radius
            m_scale.x = m_size.x / m_size0.x;
            m_scale.y = m_size.x / m_size0.x;
            // scale according to height
            m_scale.z = m_size.y / m_size0.y;
        }
        else if ( m_type == eShapeType::MESH )
        {
            // scale every dimension
            m_scale.x = m_size.x / m_size0.x;
            m_scale.y = m_size.y / m_size0.y;
            m_scale.z = m_size.z / m_size0.z;
        }
        else if ( m_type == eShapeType::HFIELD )
        {
            // not allowed, use changeElevationData instead
            std::cout << "WARNING> tried changing the scale of the collider for a hfield. Should "
                      << "change the elevation data instead" << std::endl;
        }

        m_btCollisionShape->setLocalScaling( utils::toBtVec3( m_scale ) );
    }

    void TBtCollisionAdapter::changeElevationData( const std::vector< float >& heightData )
    {
        // @todo: implement hfield functionality
    }

    extern "C" TICollisionAdapter* simulation_createCollisionAdapter( TCollision* collision )
    {
        return new TBtCollisionAdapter( collision );
    }

}