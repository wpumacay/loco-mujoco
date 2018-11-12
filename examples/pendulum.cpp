
#include "pendulum.h"

namespace pendulum
{

    mjcf::GenericElement* create( int numLinks, float linkLength )
    {
        auto _root = new mjcf::GenericElement( "mujoco" );

        // attach some general info
        {
            //includes
            auto _incVisual     = new mjcf::GenericElement( "include" );
            auto _incSkybox     = new mjcf::GenericElement( "include" );
            auto _incMaterials  = new mjcf::GenericElement( "include" );

            _incVisual->setAttributeString( "file", "visual.xml" );
            _incSkybox->setAttributeString( "file", "skybox.xml" );
            _incMaterials->setAttributeString( "file", "materials.xml" );

            _root->children.push_back( _incVisual );
            _root->children.push_back( _incSkybox );
            _root->children.push_back( _incMaterials );

            // options
            auto _options = new mjcf::GenericElement( "option" );
            _options->setAttributeFloat( "timestep", 0.02 );
            {
                auto _flagEnergy = new mjcf::GenericElement( "flag" );
                _flagEnergy->setAttributeString( "energy", "enable" );
                _options->children.push_back( _flagEnergy );
            }

            _root->children.push_back( _options );
            
        }

        auto _worldBody = mjcf::_createWorldBody();

        // attach some visual aspects, base plane and general info
        {
            // floor
            auto _floor = mjcf::_createBody( "floor" );
            auto _floorGeom = mjcf::_createGeometry( "floor", "plane", { 3, { 3.0, 3.0, 0.2 } }, 0.0f );
            _floorGeom->setAttributeString( "material", "grid" );
            _floor->children.push_back( _floorGeom );
            _worldBody->children.push_back( _floor );

            // light
            auto _light = new mjcf::GenericElement( "light" );
            _light->setAttributeString( "name", "light" );
            _light->setAttributeVec3( "pos", { 0, 0, numLinks * linkLength + 2.0f } );
            _worldBody->children.push_back( _light );

            // camera
            auto _fixedCamera = new mjcf::GenericElement( "camera" );
            _fixedCamera->setAttributeString( "name", "fixed" );
            _fixedCamera->setAttributeVec3( "pos", { 0, -1.5, 2 } );
            _fixedCamera->setAttributeArrayFloat( "xyaxes", { 6, { 1, 0, 0, 0, 1, 1 } } );
            _worldBody->children.push_back( _fixedCamera );
        }

        // create the pendulum
        {
            auto _pendulumBase = _createLinks( numLinks, linkLength );
            _worldBody->children.push_back( _pendulumBase );
        }
        
        // create the actuators
        {
            auto _actuators = new mjcf::GenericElement( "actuator" );

            for ( size_t i = 0; i < numLinks; i++ )
            {
                auto _actuator = new mjcf::GenericElement( "motor" );
                _actuator->setAttributeString( "name", std::string( "torque_" ) + std::to_string( i ) );
                _actuator->setAttributeString( "joint", std::string( PENDULUM_DEFAULT_JOINT_NAME ) + std::to_string( i ) );
                _actuator->setAttributeFloat( "gear", 1.0 );
                _actuator->setAttributeArrayFloat( "ctrlrange", { 2, { -2, 2 } } );
                _actuator->setAttributeString( "ctrllimited", "true" );

                _actuators->children.push_back( _actuator );
            }

            _root->children.push_back( _actuators );
        }

        // Attach world body
        _root->children.push_back( _worldBody );

        return _root;
    }

    mjcf::GenericElement* _createLinks( int numLinks, float linkLength )
    {
        // make the base body
        auto _baseLink = mjcf::_createBody( "base", { 0, 0, numLinks * linkLength + 0.5f } );
        // add some geometry to this element
        {
            auto _baseGeom = mjcf::_createGeometry( "base", "cylinder", 
                                                    { 1, { 0.025 } },
                                                    { 6, { 0.0, -0.1, 0.0, 0.0, 0.1, 0.0 } },
                                                    0.0f );
            _baseGeom->setAttributeString( "material", "decoration" );
            _baseLink->children.push_back( _baseGeom );
        }

        // make all other links
        auto _currentLink = _baseLink;
        
        for ( size_t i = 0; i < numLinks; i++ )
        {
            _currentLink = _createLink( _currentLink, i, linkLength );
        }

        auto _effector = mjcf::_createGeometry( "endmass", "sphere", 
                                                { 1, { 0.05f } }, 1.0f, 
                                                { 0, 0, -linkLength } );
        _currentLink->children.push_back( _effector );

        return _baseLink;
    }

    mjcf::GenericElement* _createLink( mjcf::GenericElement* parent, 
                                       int linkIndx, float linkLength )
    {
        auto _link = mjcf::_createBody( std::string( PENDULUM_DEFAULT_LINK_NAME ) + std::to_string( linkIndx ),
                                        { 0, 0, ( ( linkIndx == 0 ) ? 0.0f :-linkLength ) } );
        {
            auto _linkGeom = mjcf::_createGeometry( std::string( PENDULUM_DEFAULT_LINK_NAME ) + std::to_string( linkIndx ),
                                                    "capsule",
                                                    { 1, { 0.02 } },
                                                    { 6, { 0, 0, 0, 0, 0, -linkLength } },
                                                    0.01f );
            _linkGeom->setAttributeString( "material", "self" );
            _link->children.push_back( _linkGeom );

            auto _linkJoint = new mjcf::GenericElement( "joint" );
            _linkJoint->setAttributeString( "name", std::string( PENDULUM_DEFAULT_JOINT_NAME ) + std::to_string( linkIndx ) );
            _linkJoint->setAttributeString( "type", "hinge" );
            _linkJoint->setAttributeVec3( "axis", { 0, 1, 0 } );
            _linkJoint->setAttributeFloat( "damping", 0.1 );

            _link->children.push_back( _linkJoint );
        }

        parent->children.push_back( _link );

        return _link;
    }

}