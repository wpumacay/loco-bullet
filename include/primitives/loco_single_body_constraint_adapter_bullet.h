#pragma once

#include <loco_common_bullet.h>
#include <primitives/loco_single_body_constraint_adapter.h>

namespace loco {
namespace primitives {
    class TSingleBodyConstraint;
}}

namespace loco {
namespace primitives {

    class TIBulletSingleBodyConstraintAdapter
    {
    public :

        TIBulletSingleBodyConstraintAdapter();

        virtual ~TIBulletSingleBodyConstraintAdapter();

        void SetBulletRigidBody( btRigidBody* rigid_body_ref ) { m_BulletBodyRef = rigid_body_ref; }

        void SetBulletWorld( btDynamicsWorld* world_ref ) { m_BulletWorldRef = world_ref; }

        btRigidBody* rigid_body() { return m_BulletBodyRef; }

        const btRigidBody* rigid_body() const { return m_BulletBodyRef; }

        btDynamicsWorld* world() { return m_BulletWorldRef; }

        const btDynamicsWorld* world() const { return m_BulletWorldRef; }

        btTypedConstraint* constraint() { return m_BulletConstraint.get(); }

        const btTypedConstraint* constraint() const { return m_BulletConstraint.get(); }

    protected :

        // Reference to the bullet rigid-body (owned by single-body)
        btRigidBody* m_BulletBodyRef;
        // Reference to the bullet dynamics world (owned by simulation-obj)
        btDynamicsWorld* m_BulletWorldRef;
        // Owned bullet constraint (hinge|slider|generic6dof)
        std::unique_ptr<btTypedConstraint> m_BulletConstraint;
    };

    class TBulletSingleBodyRevoluteConstraintAdapter : public TISingleBodyRevoluteConstraintAdapter,
                                                       public TIBulletSingleBodyConstraintAdapter
    {
    public :

        TBulletSingleBodyRevoluteConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyRevoluteConstraintAdapter( constraint_ref ), TIBulletSingleBodyConstraintAdapter() {}

        TBulletSingleBodyRevoluteConstraintAdapter( const TBulletSingleBodyRevoluteConstraintAdapter& other ) = delete;

        TBulletSingleBodyRevoluteConstraintAdapter& operator= ( const TBulletSingleBodyRevoluteConstraintAdapter& other ) = delete;

        ~TBulletSingleBodyRevoluteConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetHingeAngle( TScalar hinge_angle ) override;

        void SetLimits( const TVec2& limits ) override;

        void GetHingeAngle( TScalar& dst_hinge_angle ) override;
    };

    class TBulletSingleBodyPrismaticConstraintAdapter : public TISingleBodyPrismaticConstraintAdapter,
                                                        public TIBulletSingleBodyConstraintAdapter
    {
    public :

        TBulletSingleBodyPrismaticConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyPrismaticConstraintAdapter( constraint_ref ), TIBulletSingleBodyConstraintAdapter() {}

        TBulletSingleBodyPrismaticConstraintAdapter( const TBulletSingleBodyPrismaticConstraintAdapter& other ) = delete;

        TBulletSingleBodyPrismaticConstraintAdapter& operator= ( const TBulletSingleBodyPrismaticConstraintAdapter& other ) = delete;

        ~TBulletSingleBodyPrismaticConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetSlidePosition( TScalar slide_position ) override;

        void SetLimits( const TVec2& limits ) override;

        void GetSlidePosition( TScalar& dst_slide_position ) override;
    };

    class TBulletSingleBodySphericalConstraintAdapter : public TISingleBodyConstraintAdapter,
                                                        public TIBulletSingleBodyConstraintAdapter
    {
    public :

        TBulletSingleBodySphericalConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyConstraintAdapter( constraint_ref ), TIBulletSingleBodyConstraintAdapter() {}

        TBulletSingleBodySphericalConstraintAdapter( const TBulletSingleBodySphericalConstraintAdapter& other ) = delete;

        TBulletSingleBodySphericalConstraintAdapter& operator= ( const TBulletSingleBodySphericalConstraintAdapter& other ) = delete;

        ~TBulletSingleBodySphericalConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;
    };

    class TBulletSingleBodyTranslational3dConstraintAdapter : public TISingleBodyConstraintAdapter,
                                                              public TIBulletSingleBodyConstraintAdapter
    {
    public :

        TBulletSingleBodyTranslational3dConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyConstraintAdapter( constraint_ref ), TIBulletSingleBodyConstraintAdapter() {}

        TBulletSingleBodyTranslational3dConstraintAdapter( const TBulletSingleBodyTranslational3dConstraintAdapter& other ) = delete;

        TBulletSingleBodyTranslational3dConstraintAdapter& operator= ( const TBulletSingleBodyTranslational3dConstraintAdapter& other ) = delete;

        ~TBulletSingleBodyTranslational3dConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;
    };

    class TBulletSingleBodyUniversal3dConstraintAdapter : public TISingleBodyConstraintAdapter,
                                                          public TIBulletSingleBodyConstraintAdapter
    {
    public :

        TBulletSingleBodyUniversal3dConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyConstraintAdapter( constraint_ref ), TIBulletSingleBodyConstraintAdapter() {}

        TBulletSingleBodyUniversal3dConstraintAdapter( const TBulletSingleBodyUniversal3dConstraintAdapter& other ) = delete;

        TBulletSingleBodyUniversal3dConstraintAdapter& operator= ( const TBulletSingleBodyUniversal3dConstraintAdapter& other ) = delete;

        ~TBulletSingleBodyUniversal3dConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;
    };

    class TBulletSingleBodyPlanarConstraintAdapter : public TISingleBodyConstraintAdapter,
                                                     public TIBulletSingleBodyConstraintAdapter
    {
    public :

        TBulletSingleBodyPlanarConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyConstraintAdapter( constraint_ref ), TIBulletSingleBodyConstraintAdapter() {}

        TBulletSingleBodyPlanarConstraintAdapter( const TBulletSingleBodyPlanarConstraintAdapter& other ) = delete;

        TBulletSingleBodyPlanarConstraintAdapter& operator= ( const TBulletSingleBodyPlanarConstraintAdapter& other ) = delete;

        ~TBulletSingleBodyPlanarConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;
    };
}}