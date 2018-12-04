
#pragma once

#include <vector>
#include <iostream>
#include <string>

#include <LMeshBuilder.h>

// abstract kintree (for now we are using the one concrete to bullet)
// @TODO: Should refactor to make the kintree the underlying agent structure
#include <tysocBulletKinTree.h>

namespace tysocViz
{




    /**
    * This is a wrapper on top of a kintree for our ...
    * visualizer. This will construct the engine-mesh data ...
    * to be use by the visualizer, and update these meshes ...
    * properties using the wrapped kintree (which should be ...
    * updated by the underlying physics backend.)
    */
    class TVizKinTree
    {

        private :

        std::vector< engine::LMesh* > m_meshes;



        public :




    };


}