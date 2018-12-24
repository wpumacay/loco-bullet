
/* Adapted from bullet examples. Check UrdfParser.h in Importers/ImportURDFDemo/ */

#pragma once

#include <tinyxml2.h>

#include <utils/urdf/UrdfHelpers.h>
#include <utils/urdf/UrdfSchema.h>

namespace urdf
{

    void _logError( const std::string& errorMsg );
    void _logWarning( const std::string& warningMsg );

    bool _parseTransform( btTransform& tr, tinyxml2::XMLElement* xml );
    bool _parseInertia( UrdfInertia& inertia, tinyxml2::XMLElement* config );
    bool _parseGeometry( UrdfGeometry& geom, tinyxml2::XMLElement* g );
    bool _parseVisual( UrdfModel& model, UrdfVisual& visual, tinyxml2::XMLElement* config );
    bool _parseCollision( UrdfCollision& collision, tinyxml2::XMLElement* config );
    bool _initTreeAndRoot( UrdfModel& model );
    bool _parseMaterial( UrdfMaterial& material, tinyxml2::XMLElement* config );
    bool _parseJointLimits( UrdfJoint& joint, tinyxml2::XMLElement* config );
    bool _parseJointDynamics( UrdfJoint& joint, tinyxml2::XMLElement* config );
    bool _parseJoint( UrdfJoint& joint, tinyxml2::XMLElement* config );
    bool _parseLink( UrdfModel& model, UrdfLink& link, tinyxml2::XMLElement* config );

    UrdfModel* loadGenericModel( const std::string& modelfile );

}