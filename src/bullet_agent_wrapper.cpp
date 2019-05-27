
#include <bullet_agent_wrapper.h>

namespace tysoc {
namespace bullet {

    /***************************************************************************
    *                        TBtMultiBodyLink Implementation                   *
    ****************************************************************************/

    TBtMultiBodyLink::TBtMultiBodyLink( int indx,
                                        TBtMultiBodyLink* parentLinkPtr,
                                        btMultiBody* btMultiBodyPtr,
                                        btMultiBodyDynamicsWorld* btWorldPtr )
    {
        m_btLinkIndx            = indx;
        m_parentLinkPtr         = parentLinkPtr;
        m_btMultiBodyPtr        = btMultiBodyPtr;
        m_btWorldPtr            = btWorldPtr;
        m_btCollisionShapePtr   = NULL;
        m_btLinkColliderPtr     = NULL;

        m_btJointMotor              = NULL;
        m_btSphericalJointMotor     = NULL;
        m_btJointLimitConstraint    = NULL;
    }

    TBtMultiBodyLink::~TBtMultiBodyLink()
    {
        m_parentLinkPtr         = NULL;
        m_btMultiBodyPtr        = NULL;
        m_btWorldPtr            = NULL;
        m_btCollisionShapePtr   = NULL;
        m_btLinkColliderPtr     = NULL;

        m_btJointMotor              = NULL;
        m_btSphericalJointMotor     = NULL;
        m_btJointLimitConstraint    = NULL;
    }

    void TBtMultiBodyLink::setupLinkInMultiBody( const std::string& shapeType,
                                                 const TVec3& shapeSize,
                                                 const TMat4& worldTransform,
                                                 const TMat4& localTransform,
                                                 const std::string& jointType,
                                                 const TVec3& jointAxis,
                                                 const TVec3& jointPivot,
                                                 float lowerLimit,
                                                 float upperLimit,
                                                 bool useMotor )
    {
        // grab parent index for latex usage 
        int _linkParentIndx = ( m_parentLinkPtr != NULL ) ? 
                                    m_parentLinkPtr->getIndx() : -1;

        // first just create the shape required for the link
        m_btCollisionShapePtr = utils::createCollisionShape( shapeType, shapeSize );

        // compute inertial properties from this shape
        btScalar _linkMass = utils::computeVolumeFromShape( m_btCollisionShapePtr ) * TYSOC_DEFAULT_DENSITY;
        btVector3 _linkInertia = btVector3( 0., 0., 0. );
        if ( _linkMass != 0.0f )
            m_btCollisionShapePtr->calculateLocalInertia( _linkMass, _linkInertia );

        // transform everything to bullet data types
        btTransform _btWorldTransform = utils::toBtTransform( worldTransform );
        btTransform _btLocalTransform = utils::toBtTransform( localTransform );
        btVector3 _btJointPivot = utils::toBtVec3( jointPivot );
        btVector3 _btJointAxis  = utils::toBtVec3( jointAxis );

        // grab localPos and localRot from localTransform. Recall this=local, ...
        // which is respect to the parent link. Also, get the inverse of these ...
        // quantities, as the setupXYZ methods required them
        auto _this2parent_quat  = _btLocalTransform.getRotation();
        auto _this2parent_pos   = _btLocalTransform.getOrigin();
        auto _parent2this_quat  = _this2parent_quat.inverse();
        auto _parent2this_pos   = -quatRotate( _parent2this_quat, _this2parent_pos );

        // compute pivot w.r.t. parent COM
        auto _pivot2parent_pos = _this2parent_pos - quatRotate( _this2parent_quat, -_btJointPivot );

        // setup the constraint for this link
        if ( jointType == "hinge" || jointType == "continuous" || jointType == "revolute" )
        {
            if ( shapeType == "none" && !useMotor )
            {
                // @HACK: Add a small amount of inertia to avoid erratic behaviour
                _linkMass = 0.1;
                _linkInertia = { 0.001, 0.001, 0.001 };
            }

            m_btMultiBodyPtr->setupRevolute( m_btLinkIndx, 
                                             _linkMass, 
                                             _linkInertia,
                                             _linkParentIndx,
                                             _parent2this_quat,
                                             _btJointAxis,
                                             _pivot2parent_pos,
                                             -_btJointPivot,
                                             true );

            m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_jointLowerLimit = lowerLimit;
            m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_jointUpperLimit = upperLimit;

            if ( useMotor )
            {
                m_btJointMotor = new btMultiBodyJointMotor( m_btMultiBodyPtr, 
                                                            m_btLinkIndx, 
                                                            0, 0, 5. );
                m_btJointMotor->setErp( 0.1 );
                m_btJointMotor->setPositionTarget( 0.25 );
            }
            else if ( lowerLimit <= upperLimit )
            {
                m_btJointLimitConstraint = new btMultiBodyJointLimitConstraint( 
                                                        m_btMultiBodyPtr, 
                                                        m_btLinkIndx, 
                                                        lowerLimit, 
                                                        upperLimit );
            }
        }
        else if ( jointType == "slider" || jointType == "slide" || jointType == "prismatic" )
        {
            m_btMultiBodyPtr->setupPrismatic( m_btLinkIndx,
                                              _linkMass,
                                              _linkInertia,
                                              _linkParentIndx,
                                              _parent2this_quat,
                                              _btJointAxis,
                                              _pivot2parent_pos,
                                              -_btJointPivot,
                                              true );

            m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_jointLowerLimit = lowerLimit;
            m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_jointUpperLimit = upperLimit;

            if ( useMotor )
            {
                m_btJointMotor = new btMultiBodyJointMotor( m_btMultiBodyPtr, 
                                                            m_btLinkIndx, 
                                                            0, 0, 5. );
                m_btJointMotor->setErp( 0.1 );
                m_btJointMotor->setPositionTarget( 0. );

                m_btWorldPtr->addMultiBodyConstraint( m_btJointMotor );
            }
            else if ( lowerLimit <= upperLimit )
            {
                m_btJointLimitConstraint = new btMultiBodyJointLimitConstraint( 
                                                        m_btMultiBodyPtr, 
                                                        m_btLinkIndx, 
                                                        lowerLimit, 
                                                        upperLimit );

                m_btWorldPtr->addMultiBodyConstraint( m_btJointLimitConstraint );
            }
        }
        else if ( jointType == "ball" || jointType == "spheric" || jointType == "spherical" )
        {
            m_btMultiBodyPtr->setupSpherical( m_btLinkIndx,
                                              _linkMass,
                                              _linkInertia,
                                              _linkParentIndx,
                                              _parent2this_quat,
                                              _pivot2parent_pos,
                                              -_btJointPivot,
                                              true );

            m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_jointLowerLimit = lowerLimit;
            m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_jointUpperLimit = upperLimit;

            if ( useMotor )
            {
                m_btSphericalJointMotor = new btMultiBodySphericalJointMotor( 
                                                        m_btMultiBodyPtr, 
                                                        m_btLinkIndx, 
                                                        5. );
                m_btSphericalJointMotor->setErp( 0.1 );
                m_btSphericalJointMotor->setPositionTarget( { 0., 0., 0., 1. } );

                m_btWorldPtr->addMultiBodyConstraint( m_btSphericalJointMotor );
            }
        }
        else if ( jointType == "fixed" )
        {
            m_btMultiBodyPtr->setupFixed( m_btLinkIndx,
                                          _linkMass,
                                          _linkInertia,
                                          _linkParentIndx,
                                          _parent2this_quat,
                                          _pivot2parent_pos,
                                          -_btJointPivot,
                                          true );
        }
        else
        {
            std::cout << "ERROR> joint type: " << jointType << " not supported" << std::endl;
        }

        // collider managment
        m_btLinkColliderPtr = new btMultiBodyLinkCollider( m_btMultiBodyPtr, m_btLinkIndx );
        m_btLinkColliderPtr->setCollisionShape( m_btCollisionShapePtr );
        m_btMultiBodyPtr->getLink( m_btLinkIndx ).m_collider = m_btLinkColliderPtr;

        // initialize worldTransform from parent
        m_btLinkColliderPtr->setWorldTransform( utils::toBtTransform( worldTransform ) );
    }

    TMat4 TBtMultiBodyLink::getWorldTransform()
    {
        auto _linkPos = m_btMultiBodyPtr->localPosToWorld( m_btLinkIndx, { 0., 0., 0. } );
        auto _linkRot = m_btMultiBodyPtr->localFrameToWorld( m_btLinkIndx, btMatrix3x3::getIdentity() );
        auto _linkWorldTransform = btTransform( _linkRot, _linkPos );

        return utils::fromBtTransform( _linkWorldTransform );
    }

    int TBtMultiBodyLink::getIndx()
    {
        return m_btLinkIndx;
    }

    /***************************************************************************
    *                        TBodyCompound Implementation                      *
    ****************************************************************************/

    TBodyCompound::TBodyCompound( agent::TKinTreeBody* kinTreeBodyPtr,
                                  btMultiBody* btMultiBodyPtr,
                                  btMultiBodyDynamicsWorld* btWorldPtr,
                                  TBodyCompound* parent,
                                  int linkIndx )
    {
        m_kinTreeBodyPtr    = kinTreeBodyPtr;
        m_btMultiBodyPtr    = btMultiBodyPtr;
        m_btWorldPtr        = btWorldPtr;
        m_parent            = parent;
        m_linkBaseIndx      = linkIndx;
        m_hasMultidof       = false;

        if ( kinTreeBodyPtr )
        {
            m_hasMultidof = kinTreeBodyPtr->childJoints.size() > 1;
            _constructLinksInCompound();
        }
    }

    TBodyCompound::~TBodyCompound()
    {
        m_kinTreeBodyPtr    = NULL;
        m_btMultiBodyPtr    = NULL;
        m_parent            = NULL;

        m_children.clear();

        for ( size_t q = 0; q < m_linksInChain.size(); q++ )
        {
            delete m_linksInChain[q];
            m_linksInChain[q] = NULL;
        }
        m_linksInChain.clear();
    }

    void TBodyCompound::_constructLinksInCompound()
    {
        // Grab the parent link
        TBtMultiBodyLink* _parentLink = m_parent->lastLink();

        // if the parent link is NULL, then the parent compound was the base
        bool _isParentBase = ( _parentLink == NULL );

        // grab some resources we will need
        auto _joints        = m_kinTreeBodyPtr->childJoints;
        auto _collisions    = m_kinTreeBodyPtr->childCollisions;

        // declare some transforms we will need (will define them later appropriately)
        TMat4 _trThisCompoundToParentCompound;
        TMat4 _trParentEndLinkToParentCompound;
        TMat4 _trParentEndLinkToWorld;

        // define these transforms appropriately
        if ( _isParentBase )
        {
            _trThisCompoundToParentCompound.setIdentity();
            _trParentEndLinkToParentCompound.setIdentity();
            _trParentEndLinkToWorld = m_parent->getWorldTransform();
        }
        else
        {
            _trThisCompoundToParentCompound = m_kinTreeBodyPtr->relTransform;
            _trParentEndLinkToParentCompound = m_parent->lastToBaseTransform();
            _trParentEndLinkToWorld = _parentLink->getWorldTransform();
        }

        // If dealing with a multidof case, create dummies accordingly
        if ( m_hasMultidof )
        {
            for ( size_t q = 0; q < _joints.size(); q++ )
            {
                /* Compute the transforms required to setup the link **********/
                TMat4 _trThisLinkToParentLink;
                TMat4 _trThisLinkToWorld;

                if ( q == 0 )
                {
                    _trThisLinkToParentLink = _trParentEndLinkToParentCompound.inverse() *
                                              _trThisCompoundToParentCompound *
                                              _joints[q]->relTransform;
                    _trThisLinkToWorld = _trParentEndLinkToWorld * _trThisLinkToParentLink;
                }
                else
                {
                    _trThisLinkToParentLink = _joints[q-1]->relTransform.inverse() *
                                              _joints[q]->relTransform;
                    _trThisLinkToWorld = _parentLink->getWorldTransform() * 
                                         _trThisLinkToParentLink;
                }
                /**************************************************************/

                // The pivot coincides with th origin of the link's frame
                TVec3 _jointPivotInThisLink = { 0., 0., 0. };
                // Recall, for this case joint-frame = link-frame
                TVec3 _jointAxisInThisLink = _joints[q]->axis;

                // Create the dummy link
                auto _link = new TBtMultiBodyLink( m_linkBaseIndx + m_linksInChain.size(),
                                                   _parentLink,
                                                   m_btMultiBodyPtr,
                                                   m_btWorldPtr );
                _link->setupLinkInMultiBody( "none",
                                             { 0., 0., 0. },
                                             _trThisLinkToWorld,
                                             _trThisLinkToParentLink,
                                             _joints[q]->type,
                                             _jointAxisInThisLink,
                                             _jointPivotInThisLink,
                                             _joints[q]->lowerLimit,
                                             _joints[q]->upperLimit );

                // cache this new link in the chain of this compound
                m_linksInChain.push_back( _link );

                // book keeping for next iteration
                _parentLink = _link;
            }
        }
        else
        {
            /* 
            *   Create first link from the joint dof and the collider, others ...
            *   (if any) are created later with fixed constraints
            */

            // @TODO: Check the case when there is no joints. This is a weird case ... 
            // butsome models might have this requirement (If so, why not just prune ...
            // the model to latest body with dofs and add these as geoms???).

            /* Compute the transforms required to setup this link *************/
            TMat4 _trThisLinkToParentLink;
            TMat4 _trThisLinkToWorld;
            TMat4 _trJointFrameToThisLink;

            _trThisLinkToParentLink = _trParentEndLinkToParentCompound.inverse() *
                                      _trThisCompoundToParentCompound *
                                      _collisions.front()->relTransform;

            _trThisLinkToWorld = _trParentEndLinkToWorld * _trThisLinkToParentLink;
            _trJointFrameToThisLink = _collisions.front()->relTransform.inverse() *
                                      _joints.front()->relTransform;

            /******************************************************************/

            // The pivot coincides with th origin of the joint-frame
            TVec3 _jointPivotInThisLink = _trJointFrameToThisLink.getPosition();
            // Transform axis given in joint-frame to link-frame
            TVec3 _jointAxisInThisLink = _trJointFrameToThisLink.getRotation() * 
                                         _joints.front()->axis;

            // Create the link
            auto _link = new TBtMultiBodyLink( m_linkBaseIndx + m_linksInChain.size(),
                                               _parentLink,
                                               m_btMultiBodyPtr,
                                               m_btWorldPtr );
            _link->setupLinkInMultiBody( _collisions.front()->geometry.type,
                                         _collisions.front()->geometry.size,
                                         _trThisLinkToWorld,
                                         _trThisLinkToParentLink,
                                         _joints.front()->type,
                                         _jointAxisInThisLink,
                                         _jointPivotInThisLink,
                                         _joints.front()->lowerLimit,
                                         _joints.front()->upperLimit );

            // cache this new link in the chain of this compound
            m_linksInChain.push_back( _link );

            // book keeping for next iterations
            _parentLink = _link;
        }

        // Create remaining links, which are fixed one to another
        for ( size_t q = 1; q < _collisions.size(); q++ )
        {
            /* Compute the transforms required to setup this link *************/
            TMat4 _trThisLinkToParentLink;
            TMat4 _trThisLinkToWorld;

            _trThisLinkToParentLink = _collisions[q-1]->relTransform.inverse() *
                                      _collisions[q]->relTransform;
            _trThisLinkToWorld = _parentLink->getWorldTransform() * 
                                 _trThisLinkToParentLink;
            /******************************************************************/

            // Create the link
            auto _link = new TBtMultiBodyLink( m_linkBaseIndx + m_linksInChain.size(),
                                               _parentLink,
                                               m_btMultiBodyPtr,
                                               m_btWorldPtr );
            _link->setupLinkInMultiBody( _collisions[q]->geometry.type,
                                         _collisions[q]->geometry.size,
                                         _trThisLinkToWorld,
                                         _trThisLinkToParentLink,
                                         "fixed",
                                         { 1., 0., 0. },
                                         { 0., 0., 0. } );

            // cache this new link in the chain of this compound
            m_linksInChain.push_back( _link );

            // book keeping for next iteration
            _parentLink = _link;
        }
    }

    void TBodyCompound::addChild( TBodyCompound* childCompound )
    {
        m_children.push_back( childCompound );
    }

    std::vector< TBodyCompound* > TBodyCompound::children()
    {
        return m_children;
    }

    void TBodyCompound::setWorldTransform( const TMat4& transform )
    {
        m_worldTransform = transform;
    }

    TMat4 TBodyCompound::getWorldTransform()
    {
        return m_worldTransform;
    }

    void TBodyCompound::updateTransforms()
    {
        // @WIP: Should update the kintree-body worldTransform appropriately
    }

    size_t TBodyCompound::getNumLinks()
    {
        return m_linksInChain.size();
    }

    TBtMultiBodyLink* TBodyCompound::firstLink()
    {
        if ( m_linksInChain.size () > 0 )
            return m_linksInChain.front();
        
        return NULL;
    }

    TBtMultiBodyLink* TBodyCompound::lastLink()
    {
        if ( m_linksInChain.size () > 0 )
            return m_linksInChain.back();

        return NULL;
    }

    TMat4 TBodyCompound::firstToBaseTransform()
    {
        return m_firstLinkToBaseTransform;
    }

    TMat4 TBodyCompound::baseToFirstTransform()
    {
        return m_baseToFirstLinkTransform;
    }

    TMat4 TBodyCompound::lastToBaseTransform()
    {
        return m_lastLinkToBaseTransform;
    }

    /***************************************************************************
    *                   TBtKinTreeAgentWrapper Implementation                  *
    ****************************************************************************/

    TBtKinTreeAgentWrapper::TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                                    const std::string& workingDir )
        : TKinTreeAgentWrapper( kinTreeAgentPtr, workingDir )
    {
        m_btWorldPtr        = NULL;
        m_btMultiBodyPtr    = NULL;
        m_rootCompound      = NULL;
        m_baseCompound      = NULL;
        m_currentLinkIndx   = 0;
    }

    TBtKinTreeAgentWrapper::~TBtKinTreeAgentWrapper()
    {
        if ( m_baseCompound )
            delete m_baseCompound;

        for ( size_t q = 0; q < m_bodyCompoundsArray.size(); q++ )
            delete m_bodyCompoundsArray[q];

        m_btWorldPtr    = NULL;
        m_rootCompound  = NULL;
        m_baseCompound  = NULL;

        m_bodyCompoundsMap.clear();
        m_bodyCompoundsArray.clear();

        // @TODO|@CHECK: Define the proper functionality for the destructor
    }

    void TBtKinTreeAgentWrapper::setBtWorld( btMultiBodyDynamicsWorld* btWorldPtr )
    {
        m_btWorldPtr = btWorldPtr;
    }

    void TBtKinTreeAgentWrapper::_initializeInternal()
    {
        _createBtResourcesFromKinTree();
    }

    void TBtKinTreeAgentWrapper::_resetInternal()
    {
        if ( m_kinTreeAgentPtr )
            m_kinTreeAgentPtr->reset();
    }

    void TBtKinTreeAgentWrapper::_preStepInternal()
    {
        // TODO: map user actuation commands appropriately
    }

    void TBtKinTreeAgentWrapper::_postStepInternal()
    {
        // Update each body world transform from its compound wrapper
        for ( size_t q = 0; q < m_bodyCompoundsArray.size(); q++ )
            m_bodyCompoundsArray[q]->updateTransforms();
    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromKinTree()
    {
        if ( !m_kinTreeAgentPtr )
            return;

        // define some required data for the multibody creation
        auto _numLinks      = utils::calculateNumOfLinksForMultibody( m_kinTreeAgentPtr );
        auto _isBaseFixed   = utils::shouldBaseBeFixed( m_kinTreeAgentPtr );
        auto _canSleep      = false;
        auto _baseMass      = 0.f;
        auto _baseInertia   = btVector3( 0., 0., 0. );

        // Create the btMultiBody that will represent the simulated kintree
        m_btMultiBodyPtr = new btMultiBody( _numLinks,
                                            _baseMass,
                                            _baseInertia,
                                            _isBaseFixed,
                                            _canSleep );
        m_btWorldPtr->addMultiBody( m_btMultiBodyPtr );

        // Create a dummy compound to represent the base (to propagate the starting position)
        m_baseCompound = new TBodyCompound( NULL,
                                            m_btMultiBodyPtr,
                                            m_btWorldPtr,
                                            NULL,
                                            -1 );

        // Recall we are using this dummy compound to propagate the starting ...
        // position and orientation, so grab and set this info from the kintree
        m_baseCompound->setWorldTransform( TMat4( m_kinTreeAgentPtr->getPosition(),
                                                  m_kinTreeAgentPtr->getRotation() ) );

        m_rootCompound = _createBodyCompoundFromBodyNode( m_kinTreeAgentPtr->getRootBody(), 
                                                          m_baseCompound );
    }

    TBodyCompound* TBtKinTreeAgentWrapper::_createBodyCompoundFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                            TBodyCompound* parentBodyCompound )
    {
        auto _bodyCompound = new TBodyCompound( kinTreeBodyPtr,
                                                m_btMultiBodyPtr,
                                                m_btWorldPtr,
                                                parentBodyCompound,
                                                m_currentLinkIndx );

        // save it in the storage structures appropriately
        m_bodyCompoundsArray.push_back( _bodyCompound );
        m_bodyCompoundsMap[ kinTreeBodyPtr->name ] = _bodyCompound;

        // Be nice to other compounds and update the current link index
        m_currentLinkIndx += _bodyCompound->getNumLinks();

        auto _childBodies = kinTreeBodyPtr->childBodies;
        for ( size_t q = 0; q < _childBodies.size(); q++ )
        {
            auto _childCompound = _createBodyCompoundFromBodyNode( _childBodies[q],
                                                                   _bodyCompound );

            _bodyCompound->addChild( _childCompound );
        }

        return _bodyCompound;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir )
    {
        return new TBtKinTreeAgentWrapper( kinTreeAgentPtr, workingDir );
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromFile( const std::string& name,
                                                           const std::string& filename,
                                                           const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromId( const std::string& name,
                                                         const std::string& format,
                                                         const std::string& id,
                                                         const std::string& workingDir )
    {
        // @WIP
        return NULL;
    }


}}