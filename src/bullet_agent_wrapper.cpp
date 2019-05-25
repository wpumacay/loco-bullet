
#include <bullet_agent_wrapper.h>

namespace tysoc {
namespace bullet {

    TBtKinTreeAgentWrapper::TBtKinTreeAgentWrapper( agent::TAgentKinTree* kinTreeAgentPtr,
                                                    const std::string& workingDir )
        : TKinTreeAgentWrapper( kinTreeAgentPtr, workingDir )
    {
        m_rootCompound = NULL;
        m_btWorldPtr = NULL;
    }

    TBtKinTreeAgentWrapper::~TBtKinTreeAgentWrapper()
    {
        m_btWorldPtr = NULL;
        m_rootCompound = NULL;

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
        
    }

    void TBtKinTreeAgentWrapper::_postStepInternal()
    {

    }

    void TBtKinTreeAgentWrapper::_createBtResourcesFromKinTree()
    {
        if ( !m_kinTreeAgentPtr )
            return;

        m_rootCompound = _createBodyCompoundFromBodyNode( m_kinTreeAgentPtr->getRootBody(), NULL );
    }

    TBodyCompound* TBtKinTreeAgentWrapper::_createBodyCompoundFromBodyNode( agent::TKinTreeBody* kinTreeBodyPtr,
                                                                            TBodyCompound* parentBodyCompound )
    {
        return NULL;
    }

    extern "C" TKinTreeAgentWrapper* agent_createFromAbstract( agent::TAgentKinTree* kinTreeAgentPtr,
                                                               const std::string& workingDir )
    {
        // @WIP
        return NULL;
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