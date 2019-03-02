
#pragma once

// interface for our VIZ implementation
#include <viz/viz.h>
// include mujoco functionality
#include <mujoco.h>
// and the glfw3 version provided by mujoco
#include <glfw3.h>
// and the current UI functionality
#include <mujocoviz_ui.h>

namespace tysoc {
namespace viz {


    class TMujocoVisualizer : public TIVisualizer
    {

        private :

        mjModel*    m_mjcModelPtr;
        mjData*     m_mjcDataPtr;
        mjvScene*   m_mjcScenePtr;
        mjrContext* m_mjcGpuContextPtr;
        mjvCamera*  m_mjcCameraPtr;
        mjvOption*  m_mjcOptionPtr;

        GLFWwindow* m_glfwWindowPtr;

        bool m_singleKeys[1024];// to check single click presses

        struct
        {
            bool buttonLeftPressed;
            bool buttonMiddlePressed;
            bool buttonRightPressed;
            double lastX;
            double lastY;
        } m_demoInfo;

        // the UI context
        TMujocoContextUI*   m_uiContextPtr;

        static TMujocoVisualizer* _INSTANCE;

        void _onKeyCallback( int keyCode, int action, int modifiers );
        void _onMouseButtonCallback( int button, int action, int modifiers );
        void _onMouseMoveCallback( double x, double y );
        void _onScrollCallback( double xOffset, double yOffset );

        protected :

        bool _initializeInternal() override;
        void _updateInternal() override;
        void _renderUIInternal() override;
        bool _isActiveInternal() override;

        int _remapKeyInternal( int keyCode ) override;
        bool _isKeyDownInternal( int keyCode ) override;
        bool _checkSingleKeyPressInternal( int keyCode ) override;

        TIVizCamera* _createCameraInternal( const std::string& name,
                                            const std::string& type,
                                            const TVec3& pos,
                                            const TMat3& rot ) override;
        void _changeToCameraInternal( TIVizCamera* cameraPtr ) override;
        void _grabCameraFrameInternal( TIVizCamera* cameraPtr,
                                       TIVizTexture& rgbTexture,
                                       TIVizTexture& depthTexture ) override;

        TIVizLight* _createLightInternal( const std::string& name,
                                          const std::string& type,
                                          const TVec3& pos ) override;

        void _collectSimPayloadInternal( void* payload, const std::string& type ) override;

        public :

        TMujocoVisualizer( TScenario* scenarioPtr,
                           const std::string& workingDir );
        ~TMujocoVisualizer(); // @CHECK: check for virtual destructors

        void setMjcModel( mjModel* mjcModelPtr );
        void setMjcData( mjData* mjcDataPtr );
        void setMjcScene( mjvScene* mjcScenePtr );
        void setMjcCamera( mjvCamera* mjcCameraPtr );
        void setMjcOption( mjvOption* mjcOptionPtr );

        static void KeybackCallback( GLFWwindow* windowPtr, 
                                     int keyCode,
                                     int scanCode,
                                     int action,
                                     int modifiers );

        static void MouseButtonCallback( GLFWwindow* windowPtr,
                                         int button,
                                         int action,
                                         int modifiers );

        static void MouseMoveCallback( GLFWwindow* windowPtr,
                                       double x,
                                       double y );

        static void ScrollCallback( GLFWwindow* windowPtr,
                                    double xOffset,
                                    double yOffset );
    };

    extern "C" TIVisualizer* visualizer_create( TScenario* scenarioPtr,
                                                const std::string& workingDir );

}}