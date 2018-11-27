
#pragma once

#include <tysocBulletCommon.h>
#include <agent/agent.h>

namespace tysocBullet
{

    class TBulletAgentWrapper
    {

        private :

        std::string m_name;

        tysocagent::TAgent* m_agentPtr;

        void _createWrappedAgentObj();

        float m_startX;
        float m_startY;
        float m_startZ;

        public :

        TBulletAgentWrapper( const std::string& name,
                             float posX, float posY, float posZ );
        ~TBulletAgentWrapper();

        std::string name();
        tysocagent::TAgent* agent() { return m_agentPtr; }

        void getPosition( float &x, float &y, float &z );
        void setPosition( float x, float y, float z );

        void preStep();// actuator controls
        void postStep();// updates the bodies and joints
    };

}