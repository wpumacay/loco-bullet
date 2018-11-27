
#pragma once

#include <tysocBulletTerrain.h>
#include <tysocBulletAgent.h>

#include <dirent.h>

// @TODO: Add comments-docs for doxygen

namespace tysocBullet
{

    class TGenericParams
    {
        private :

        std::map< std::string, int > m_ints;
        std::map< std::string, float > m_floats;
        std::map< std::string, std::string > m_strings;

        public :

        void set( const std::string& name, int val )
        {
            m_ints[ name ] = val;
        }

        void set( const std::string& name, float val )
        {
            m_floats[ name ] = val;
        }

        void set( const std::string& name, const std::string& str )
        {
            m_strings[ name ] = str;
        }

        int getInt( const std::string& name, int def = 0 ) const
        {
            if ( m_ints.find( name ) != m_ints.end() )
            {
                return m_ints.at( name );
            }
            return def;
        }

        float getFloat( const std::string& name, float def = 0.0f ) const
        {
            if ( m_floats.find( name ) != m_floats.end() )
            {
                return m_floats.at( name );
            }
            return def;
        }

        std::string getString( const std::string& name, const std::string& def = "undefined" ) const
        {
            if ( m_strings.find( name ) != m_strings.end() )
            {
                return m_strings.at( name );
            }
            return def;
        }

    };


    class TBulletFactory
    {
        private :


        public :

        TBulletFactory();
        ~TBulletFactory();

        TBulletTerrainGenWrapper* createTerrainGen( const std::string& name,
                                                    const std::string& type,
                                                    const TGenericParams& params );

    };



}