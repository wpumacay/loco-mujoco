
#include <mujoco_sandbox_body_wrapper.h>

namespace tysoc {
namespace mujoco {


    TMjcBodyWrapper::TMjcBodyWrapper( sandbox::TBody* bodyPtr,
                                      const std::string& workingDir )
        : TBodyWrapper( bodyPtr, workingDir )
    {
        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;

        // create the mjcf resources element for this agent
        m_mjcfResourcesPtr = new mjcf::GenericElement( "mujoco" );
        m_mjcfResourcesPtr->setAttributeString( "model", name() );
        // and declare the reference to the target mjcf (where to place the resources)
        m_mjcfTargetResourcesPtr = NULL;

        // collect the required data
        _createMjcResourcesFromBody();
    }

    TMjcBodyWrapper::~TMjcBodyWrapper()
    {
        if ( m_mjcfResourcesPtr )
        {
            delete m_mjcfResourcesPtr;
            m_mjcfResourcesPtr = NULL;
        }

        m_mjcModelPtr   = NULL;
        m_mjcDataPtr    = NULL;
        m_mjcScenePtr   = NULL;
        m_mjcfTargetResourcesPtr = NULL;
    }

    void TMjcBodyWrapper::setMjcModel( mjModel* mjcModelPtr )
    {
        m_mjcModelPtr = mjcModelPtr;
    }

    void TMjcBodyWrapper::setMjcData( mjData* mjcDataPtr )
    {
        m_mjcDataPtr = mjcDataPtr;
    }

    void TMjcBodyWrapper::setMjcScene( mjvScene* mjcScenePtr )
    {
        m_mjcScenePtr = mjcScenePtr;
    }

    void TMjcBodyWrapper::setMjcfTargetElm( mjcf::GenericElement* targetResourcesPtr )
    {
        m_mjcfTargetResourcesPtr = targetResourcesPtr;
    }

    void TMjcBodyWrapper::_initializeInternal()
    {
        // Check if the caller (TMjcSimulation) set the target reference
        if ( !m_mjcfTargetResourcesPtr )
        {
            std::cout << "ERROR> mjc-sim object must pass a reference of the"
                      << " target resources to this agent" << std::endl;
            return;
        }

        // grab the mjcf resources to inject, namely the worlbody
        auto _worldBodyElmPtr = mjcf::findFirstChildByType( m_mjcfResourcesPtr, "worldbody" );

        if ( _worldBodyElmPtr )
            m_mjcfTargetResourcesPtr->children.push_back( _worldBodyElmPtr );
    }

    void TMjcBodyWrapper::_resetInternal()
    {
        if ( !m_bodyPtr )
            return;

        m_bodyPtr->worldTransform.setPosition( m_posStart );
        m_bodyPtr->worldTransform.setRotation( m_rotStart );
    }

    void TMjcBodyWrapper::_preStepInternal()
    {
        // Nothing to do here
    }

    void TMjcBodyWrapper::_postStepInternal()
    {
        if ( !m_bodyPtr )
            return;

        // grab the position from the mujoco backend
        auto _pos = utils::getBodyPosition( m_mjcModelPtr,
                                            m_mjcDataPtr,
                                            m_bodyPtr->name );
        // and the rotation as well
        auto _rot = utils::getBodyOrientation( m_mjcModelPtr,
                                               m_mjcDataPtr,
                                               m_bodyPtr->name );

        // then set it to the body's worldTransform
        m_bodyPtr->worldTransform.setPosition( _pos );
        m_bodyPtr->worldTransform.setRotation( _rot );
    }

    void TMjcBodyWrapper::_changePositionInternal()
    {
        if ( !m_bodyPtr )
            return;

        utils::setBodyPosition( m_mjcModelPtr,
                                m_mjcDataPtr,
                                m_bodyPtr->name,
                                m_bodyPtr->worldTransform.getPosition() );
    }

    void TMjcBodyWrapper::_changeRotationInternal()
    {
        if ( !m_bodyPtr )
            return;

        utils::setBodyOrientation( m_mjcModelPtr,
                                   m_mjcDataPtr,
                                   m_bodyPtr->name,
                                   m_bodyPtr->worldTransform.getRotation() );
    }

    void TMjcBodyWrapper::_changeSizeInternal()
    {
        if ( !m_bodyPtr )
            return;

        utils::changeSize( m_mjcModelPtr,
                           m_bodyPtr->name,
                           m_bodyPtr->size );
    }

    void TMjcBodyWrapper::_createMjcResourcesFromBody()
    {
        if ( !m_mjcfResourcesPtr )
            return;

        if ( !m_bodyPtr )
            return;

        auto _worldBody = new mjcf::GenericElement( "worldbody" );
        m_mjcfResourcesPtr->children.push_back( _worldBody );
        
        auto _bodyElmPtr = new mjcf::GenericElement( "body" );
        _bodyElmPtr->setAttributeString( "name", m_bodyPtr->name );
        _bodyElmPtr->setAttributeVec3( "pos", m_bodyPtr->worldTransform.getPosition() );
        _bodyElmPtr->setAttributeVec4( "quat", m_bodyPtr->worldTransform.getRotQuaternion() );
        _worldBody->children.push_back( _bodyElmPtr );

        auto _geomElmPtr = new mjcf::GenericElement( "geom" );
        _geomElmPtr->setAttributeString( "name", m_bodyPtr->name );
        _geomElmPtr->setAttributeString( "type", m_bodyPtr->type );
        _geomElmPtr->setAttributeVec3( "size", _extractMjcSizeFromStandardSize( m_bodyPtr->type, m_bodyPtr->size ) );
        _bodyElmPtr->children.push_back( _geomElmPtr );

        // @DEBUG: just for testing, add free type joints
        auto _freeJointElmPtr = new mjcf::GenericElement( "joint" );
        _freeJointElmPtr->setAttributeString( "name", m_bodyPtr->name + "_jnt_free" );
        _freeJointElmPtr->setAttributeString( "type", "free" );
        _bodyElmPtr->children.push_back( _freeJointElmPtr );
    }

    TVec3 TMjcBodyWrapper::_extractMjcSizeFromStandardSize( const std::string& shape,
                                                            const TVec3& size )
    {
        TVec3 _res;

        if ( shape == "plane" )
            _res = { size.x, size.y, size.z };

        else if ( shape == "sphere" )
            _res = { size.x, size.y, size.z };

        else if ( shape == "capsule" || shape == "cylinder" )
            _res = { size.x, 0.5f * size.y, size.z };

        else if ( shape == "box" )
            _res = { 0.5f * size.x, 0.5f * size.y, 0.5f * size.z };

        return _res;
    }

}}