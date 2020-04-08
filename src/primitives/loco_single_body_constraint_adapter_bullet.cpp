
#include <primitives/loco_single_body_constraint_adapter_bullet.h>

namespace loco {
namespace bullet {

    //********************************************************************************************//
    //                              Bullet-Adapter Interface Impl.                                //
    //********************************************************************************************//

    TIBulletSingleBodyConstraintAdapter::TIBulletSingleBodyConstraintAdapter()
    {
        m_BulletBodyRef = nullptr;
        m_BulletWorldRef = nullptr;
        m_BulletConstraint = nullptr;
    }

    TIBulletSingleBodyConstraintAdapter::~TIBulletSingleBodyConstraintAdapter()
    {
        m_BulletBodyRef = nullptr;
        m_BulletWorldRef = nullptr;
        m_BulletConstraint = nullptr;
    }

    //********************************************************************************************//
    //                              Revolute-constraint Adapter Impl                              //
    //********************************************************************************************//

    TBulletSingleBodyRevoluteConstraintAdapter::~TBulletSingleBodyRevoluteConstraintAdapter()
    {
        if ( m_ConstraintRef )
            m_ConstraintRef->DetachSim();
        m_ConstraintRef = nullptr;
    }

    void TBulletSingleBodyRevoluteConstraintAdapter::Build()
    {
        auto revolute_constraint = dynamic_cast<TSingleBodyRevoluteConstraint*>( m_ConstraintRef );
        LOCO_CORE_ASSERT( revolute_constraint, "TBulletSingleBodyRevoluteConstraintAdapter::Build >>> \
                          constraint reference must be of type \"Revolute\", for constraint named {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_BulletBodyRef, "TBulletSingleBodyRevoluteConstraintAdapter::Build >>> \
                          rigid-body reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const btVector3 pivot = vec3_to_bt( TVec3( revolute_constraint->local_tf().col( 3 ) ) );
        const btVector3 axis = vec3_to_bt( TMat3( revolute_constraint->local_tf() ) * revolute_constraint->axis() );
        auto hinge_constraint = std::make_unique<btHingeConstraint>( *m_BulletBodyRef, pivot, axis );
        hinge_constraint->setLimit( 1.0f, -1.0f ); // no limits first, user changes it with "SetLimits"
        m_BulletConstraint = std::move( hinge_constraint );
    }

    void TBulletSingleBodyRevoluteConstraintAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_BulletWorldRef, "TBulletSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          bullet-world reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );
        m_BulletWorldRef->addConstraint( m_BulletConstraint.get() );
    }

    void TBulletSingleBodyRevoluteConstraintAdapter::Reset()
    {
        SetHingeAngle( 0.0f );
    }

    void TBulletSingleBodyRevoluteConstraintAdapter::OnDetach()
    {
        m_Detached = true;
        m_ConstraintRef = nullptr;
    }

    void TBulletSingleBodyRevoluteConstraintAdapter::SetHingeAngle( TScalar hinge_angle )
    {
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );

    }

    void TBulletSingleBodyRevoluteConstraintAdapter::SetLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );
        if ( auto bt_hinge_constraint = dynamic_cast<btHingeConstraint*>( m_BulletConstraint.get() ) )
            bt_hinge_constraint->setLimit( limits.x(), limits.y() );
    }

    void TBulletSingleBodyRevoluteConstraintAdapter::GetHingeAngle( TScalar& dst_hinge_angle )
    {
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyRevoluteConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );
        if ( auto bt_hinge_constraint = dynamic_cast<btHingeConstraint*>( m_BulletConstraint.get() ) )
            dst_hinge_angle = bt_hinge_constraint->getHingeAngle();
    }

    //********************************************************************************************//
    //                              Prismatic-constraint Adapter Impl                             //
    //********************************************************************************************//

    TBulletSingleBodyPrismaticConstraintAdapter::~TBulletSingleBodyPrismaticConstraintAdapter()
    {
        if ( m_ConstraintRef )
            m_ConstraintRef->DetachSim();
        m_ConstraintRef = nullptr;
    }

    void TBulletSingleBodyPrismaticConstraintAdapter::Build()
    {
        auto prismatic_constraint = dynamic_cast<TSingleBodyPrismaticConstraint*>( m_ConstraintRef );
        LOCO_CORE_ASSERT( prismatic_constraint, "TBulletSingleBodyPrismaticConstraintAdapter::Build >>> \
                          constraint reference must be of type \"Prismatic\", for constraint named {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_BulletBodyRef, "TBulletSingleBodyPrismaticConstraintAdapter::Build >>> \
                          rigid-body reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const TVec3 axis_local = prismatic_constraint->axis();
        const TVec3 axis_world = TMat3( prismatic_constraint->local_tf() ) * axis_local;
        const TVec3 x_axis_world = { 1.0f, 0.0f, 0.0f };
        const TVec4 quat_tf = ShortestArcQuat( x_axis_world, axis_world );
        const TMat3 tf_rot = tinymath::rotation( quat_tf );
        btTransform frame_constraint = btTransform::getIdentity();
        frame_constraint.setBasis( mat3_to_bt( tf_rot ) );

        auto slider_constraint = std::make_unique<btSliderConstraint>( *m_BulletBodyRef, frame_constraint, false );
        slider_constraint->setLowerLinLimit( prismatic_constraint->limits().x() );
        slider_constraint->setUpperLinLimit( prismatic_constraint->limits().y() );
        m_BulletConstraint = std::move( slider_constraint );
    }

    void TBulletSingleBodyPrismaticConstraintAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_BulletWorldRef, "TBulletSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          bullet-world reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );
        m_BulletWorldRef->addConstraint( m_BulletConstraint.get() );
    }

    void TBulletSingleBodyPrismaticConstraintAdapter::Reset()
    {
        SetSlidePosition( 0.0f );
    }

    void TBulletSingleBodyPrismaticConstraintAdapter::OnDetach()
    {
        m_Detached = true;
        m_ConstraintRef = nullptr;
    }

    void TBulletSingleBodyPrismaticConstraintAdapter::SetSlidePosition( TScalar slide_position )
    {
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );

    }

    void TBulletSingleBodyPrismaticConstraintAdapter::SetLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );
        if ( auto bt_prismatic_constraint = dynamic_cast<btSliderConstraint*>( m_BulletConstraint.get() ) )
        {
            bt_prismatic_constraint->setLowerLinLimit( limits.x() );
            bt_prismatic_constraint->setUpperLinLimit( limits.y() );
        }
    }

    void TBulletSingleBodyPrismaticConstraintAdapter::GetSlidePosition( TScalar& dst_slide_position )
    {
        LOCO_CORE_ASSERT( m_BulletConstraint, "TBulletSingleBodyPrismaticConstraintAdapter::Initialize >>> \
                          bullet typed-constraint must be created by now (got nullptr). Perhaps missing call to ->Build(), fof constraint named {0}", m_ConstraintRef->name() );
        if ( auto bt_prismatic_constraint = dynamic_cast<btSliderConstraint*>( m_BulletConstraint.get() ) )
            dst_slide_position = bt_prismatic_constraint->getLinearPos();
    }

}}