
#pragma once

#include <tinyxml2.h>

#include <utils/urdf/UrdfHelpers.h>
#include <utils/urdf/UrdfSchema.h>

namespace urdf
{

    class UrdfParser
    {
    protected:
        UrdfModel m_urdf2Model;
        btAlignedObjectArray<UrdfModel*> m_sdfModels;
        btAlignedObjectArray<UrdfModel*> m_tmpModels;

        bool m_parseSDF;
        int m_activeSdfModel;

        btScalar m_urdfScaling;

        void _logError( const std::string& errorMsg );
        void _logWarning( const std::string& warningMsg );

        bool parseTransform(btTransform& tr, tinyxml2::XMLElement* xml, bool parseSDF = false);
        bool parseInertia(UrdfInertia& inertia, tinyxml2::XMLElement* config);
        bool parseGeometry(UrdfGeometry& geom, tinyxml2::XMLElement* g);
        bool parseVisual(UrdfModel& model, UrdfVisual& visual, tinyxml2::XMLElement* config);
        bool parseCollision(UrdfCollision& collision, tinyxml2::XMLElement* config);
        bool initTreeAndRoot(UrdfModel& model);
        bool parseMaterial(UrdfMaterial& material, tinyxml2::XMLElement* config);
        bool parseJointLimits(UrdfJoint& joint, tinyxml2::XMLElement* config);
        bool parseJointDynamics(UrdfJoint& joint, tinyxml2::XMLElement* config);
        bool parseJoint(UrdfJoint& joint, tinyxml2::XMLElement* config);
        bool parseLink(UrdfModel& model, UrdfLink& link, tinyxml2::XMLElement* config);
        bool parseSensor(UrdfModel& model, UrdfLink& link, UrdfJoint& joint, tinyxml2::XMLElement* config);

    public:
        UrdfParser();
        virtual ~UrdfParser();

        void setParseSDF(bool useSDF)
        {
            m_parseSDF = useSDF;
        }
        bool getParseSDF() const
        {
            return m_parseSDF;
        }
        void setGlobalScaling(btScalar scaling)
        {
            m_urdfScaling = scaling;
        }

        bool loadUrdf(const char* filepath, bool forceFixedBase, bool parseSensors);

        bool loadUrdf(const char* filepath, bool forceFixedBase)
        {
            return loadUrdf(filepath, forceFixedBase, false);
        }

        bool loadSDF(const char* sdfText);

        int getNumModels() const
        {
            //user should have loaded an SDF when calling this method
            if (m_parseSDF)
            {
                return m_sdfModels.size();
            }
            return 1;
        }

        void activateModel(int modelIndex);

        UrdfModel& getModelByIndex(int index)
        {
            //user should have loaded an SDF when calling this method
            btAssert(m_parseSDF);

            return *m_sdfModels[index];
        }

        const UrdfModel& getModelByIndex(int index) const
        {
            //user should have loaded an SDF when calling this method
            btAssert(m_parseSDF);

            return *m_sdfModels[index];
        }

        const UrdfModel& getModel() const
        {
            if (m_parseSDF)
            {
                return *m_sdfModels[m_activeSdfModel];
            }

            return m_urdf2Model;
        }

        UrdfModel& getModel()
        {
            if (m_parseSDF)
            {
                return *m_sdfModels[m_activeSdfModel];
            }
            return m_urdf2Model;
        }

        std::string sourceFileLocation(tinyxml2::XMLElement* e);

        void setSourceFile(const std::string& sourceFile)
        {
            m_urdf2Model.m_sourceFile = sourceFile;
        }
    };

}