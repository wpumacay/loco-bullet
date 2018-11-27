
#include <tysocBulletAgent.h>




namespace tysocBullet
{


    TBulletAgentWrapper::TBulletAgentWrapper( const std::string& name,
                                              float posX, float posY, float posZ )
    {
        m_name = name;
        m_startX = posX;
        m_startY = posY;
        m_startZ = posZ;

        // create wrapped agent object
        _createWrappedAgentObj();
    }

    TBulletAgentWrapper::~TBulletAgentWrapper()
    {
        if ( m_agentPtr )
        {
            // deletion of the base reosurces is in charge of the scenario
            m_agentPtr = NULL;
        }
    }

    std::string TBulletAgentWrapper::name()
    {
        return m_name;
    }

    void TBulletAgentWrapper::_createWrappedAgentObj()
    {
        m_agentPtr = new tysocagent::TAgent( m_name );
    }

    void TBulletAgentWrapper::getPosition( float &x, float &y, float &z )
    {

    }

    void TBulletAgentWrapper::setPosition( float x, float y, float z )
    {

    }

    void TBulletAgentWrapper::preStep()
    {
        // set all ctrls from the agent object actuator values ( from user )
        auto _actuators = m_agentPtr->actuators();

        for ( auto it = _actuators.begin(); it != _actuators.end(); it++ )
        {
            // set actuators
        }
    }

    void TBulletAgentWrapper::postStep()
    {
        auto _geometries = m_agentPtr->geometries();
        for ( auto it = _geometries.begin(); it != _geometries.end(); it++ )
        {
            float _pos[3];
            float _rotmat[9];
            // @PORT: USE APPROPIATE GET_TRANSFORM FUNCTION

            it->second->pos = { _pos[0], _pos[1], _pos[2] };

            it->second->rotmat[0] = _rotmat[0];
            it->second->rotmat[1] = _rotmat[1];
            it->second->rotmat[2] = _rotmat[2];
            it->second->rotmat[3] = _rotmat[3];
            it->second->rotmat[4] = _rotmat[4];
            it->second->rotmat[5] = _rotmat[5];
            it->second->rotmat[6] = _rotmat[6];
            it->second->rotmat[7] = _rotmat[7];
            it->second->rotmat[8] = _rotmat[8];

            float _color[3];
            // @PORT: USE APPROPIATE GET_COLOR FUNCTION

            it->second->color.r = _color[0];
            it->second->color.g = _color[1];
            it->second->color.b = _color[2];
        }

        // @PORT: get joint angles here
        auto _joints = m_agentPtr->joints();

        // @PORT: get bodies information here
        auto _bodies = m_agentPtr->bodies();
        for ( auto it = _bodies.begin(); it != _bodies.end(); it++ )
        {
            float _pos[3];
            it->second->pos = { _pos[0], _pos[1], _pos[2] };
        }
    }

}