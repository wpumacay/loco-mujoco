
#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <sandbox/body_wrapper.h>

namespace tysoc {
namespace mujoco {

    class TMjcBodyWrapper : public TBodyWrapper
    {
        private :

        // mujoco data to be sent to the xml
        mjcf::GenericElement* m_mjcfResourcesPtr;
        // mjcf data where to send the data from above
        mjcf::GenericElement* m_mjcfTargetResourcesPtr;

        // creates mjcf resources for this wrapper
        void _createMjcResourcesFromBody( mjcf::GenericElement* parentElmPtr,
                                          sandbox::TBody* bodyPtr );
        // converts standard size to mjcf size
        TVec3 _extractMjcSizeFromStandardSize( const std::string& shape,
                                               const TVec3& size );

        // updates a body recursively
        void _updateBodyRecursively( sandbox::TBody* bodyPtr );

        // mujoco simulation data
        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;



        protected :

        void _initializeInternal() override;
        void _resetInternal() override;
        void _preStepInternal() override;
        void _postStepInternal() override;
        void _changePositionInternal() override;
        void _changeRotationInternal() override;
        void _changeSizeInternal() override;

        public :

        TMjcBodyWrapper( sandbox::TBody* bodyPtr,
                         const std::string& workingDir );

        ~TMjcBodyWrapper();

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );
        void setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr );
    };




}}