
#include <adapters/mujoco_compound_adapter.h>

namespace tysoc
{

    TMjcCompoundAdapter::TMjcCompoundAdapter( TCompound* compoundRef )
        : TICompoundAdapter( compoundRef )
    {
        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcfXmlResource = nullptr;
        m_mjcfXmlAssetResources = nullptr;
    }

    TMjcCompoundAdapter::~TMjcCompoundAdapter()
    {
        m_mjcModelRef = nullptr;
        m_mjcDataRef = nullptr;
        m_mjcfXmlResource = nullptr;
        m_mjcfXmlAssetResources = nullptr;
    }

    void TMjcCompoundAdapter::build()
    {
        if ( !m_compoundRef )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundAdapter::build() >>> can't construct adapter for null compound" );
            return;
        }

        if ( !m_compoundRef->root() )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundAdapter::build() >>> to build resources we require a root-body\
                               to start the traversal, but body \"{0}\" doesn't have it", m_compoundRef->name() );
            return;
        }

        auto _compoundRootBodyRef = m_compoundRef->root();
        m_mjcfXmlResource = new mjcf::GenericElement( "worldbody" );
        m_mjcfXmlAssetResources = new mjcf::GenericElement( "asset" );

        /* request bodies to build their resources recursively (DFS to update in parent->children order) */
        std::stack< TCompoundBody* > _bodiesToBuild;
        _bodiesToBuild.push( _compoundRootBodyRef );
        while ( _bodiesToBuild.size() > 0 )
        {
            auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( _bodiesToBuild.top() );
            _bodiesToBuild.pop();
            if ( !_compoundBodyRef )
                continue;
            if ( !_compoundBodyRef->adapter() )
                continue;

            auto _compoundBodyAdapterRef = dynamic_cast< TMjcCompoundBodyAdapter* >( _compoundBodyRef->adapter() );
            _compoundBodyAdapterRef->build();

            auto _children = _compoundBodyRef->children();
            for ( auto _child : _children )
                _bodiesToBuild.push( _child );
        }

        /* grab mjcf-resources from the top of the compound (root) into our mjcf 
           resource (DFS to grab in parent-children order) */
        std::stack< TCompoundBody* > _bodiesFromWhomToGrab;
        _bodiesFromWhomToGrab.push( _compoundRootBodyRef );
        while ( _bodiesFromWhomToGrab.size() > 0 )
        {
            auto _compoundBodyRef = dynamic_cast< TCompoundBody* >( _bodiesFromWhomToGrab.top() );
            _bodiesFromWhomToGrab.pop();
            if ( !_compoundBodyRef )
                continue;

            auto _compoundBodyAdapterRef = dynamic_cast< TMjcCompoundBodyAdapter* >( _compoundBodyRef->adapter() );
            auto _compoundBodyMjcfResource = _compoundBodyAdapterRef->mjcfResource();
            auto _compoundBodyMjcfAssetResources = _compoundBodyAdapterRef->mjcfAssetResources();

            if ( !_compoundBodyMjcfResource )
            {
                TYSOC_CORE_ERROR( "TMjcCompoundAdapter::build() >>> something went wrong while creating \
                                   mjcf-resources for body \"{0}\"", _compoundBodyRef->name() );
            }
            else
            {
                auto _parentBodyRef = _compoundBodyRef->parent();
                if ( _parentBodyRef )
                {
                    if ( _parentBodyRef->adapter() )
                    {
                        auto _parentBodyAdapterRef = dynamic_cast< TMjcCompoundBodyAdapter* >( _parentBodyRef->adapter() );
                        _parentBodyAdapterRef->mjcfResource()->children.push_back( _compoundBodyMjcfResource );
                    }
                }
                else
                {
                    m_mjcfXmlResource->children.push_back( _compoundBodyMjcfResource );
                }
            }

            if ( _compoundBodyMjcfAssetResources )
                m_mjcfXmlAssetResources->children.insert( m_mjcfXmlAssetResources->children.end(), 
                                                          _compoundBodyMjcfAssetResources->children.begin(),
                                                          _compoundBodyMjcfAssetResources->children.end() );

            auto _children = _compoundBodyRef->children();
            for ( auto _child : _children )
                _bodiesFromWhomToGrab.push( _child );
        }
    }

    void TMjcCompoundAdapter::reset()
    {
        // nothing to do here, as the reset is handled by the root body's adapter
    }

    void TMjcCompoundAdapter::preStep()
    {
        // nothing required here (world-pose is grabbed with getters below)
    }

    void TMjcCompoundAdapter::postStep()
    {
        // nothing required here (world-pose is grabbed with getters below)
    }

    void TMjcCompoundAdapter::setPosition( const TVec3& unused_position )
    {
        auto _compoundRootBodyRef = m_compoundRef->root();
        // compute the world-transform where the root-body should be
        auto _compoundRootBodyWorldTf = m_compoundRef->tf() * _compoundRootBodyRef->localTf();
        // set the position of the root-body to this new position
        _compoundRootBodyRef->setPosition( _compoundRootBodyWorldTf.getPosition() );
    }

    void TMjcCompoundAdapter::setRotation( const TMat3& unused_rotation )
    {
        auto _compoundRootBodyRef = m_compoundRef->root();
        // compute the world-transform where the root-body should be
        auto _compoundRootBodyWorldTf = m_compoundRef->tf() * _compoundRootBodyRef->localTf();
        // set the position of the root-body to this new position
        _compoundRootBodyRef->setRotation( _compoundRootBodyWorldTf.getRotation() );
    }

    void TMjcCompoundAdapter::setTransform( const TMat4& unused_transform )
    {
        auto _compoundRootBodyRef = m_compoundRef->root();
        // compute the world-transform where the root-body should be
        auto _compoundRootBodyWorldTf = m_compoundRef->tf() * _compoundRootBodyRef->localTf();
        // set the position of the root-body to this new position
        _compoundRootBodyRef->setTransform( _compoundRootBodyWorldTf );
    }

    void TMjcCompoundAdapter::getPosition( TVec3& dstPosition )
    {
        auto _compoundRootBodyRef = m_compoundRef->root();
        if ( !_compoundRootBodyRef->adapter() )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundAdapter::getPosition() >>> the position of compound \"{0}\" \
                               can only be grabbed using the root's adapter getTransform() method", m_compoundRef->name() );
            return;
        }

        auto _compoundRootAdapterRef = dynamic_cast< TMjcCompoundBodyAdapter* >( _compoundRootBodyRef->adapter() );
        // grab the world-transform of the root-body (base is just a local-transform away from the root)
        auto _compoundRootBodyWorldTf = TMat4();
        _compoundRootAdapterRef->getTransform( _compoundRootBodyWorldTf );
        // compute the position of the base of the compound, using the root's transform
        dstPosition = ( _compoundRootBodyWorldTf * ( _compoundRootBodyRef->localTf() ).inverse() ).getPosition();
    }

    void TMjcCompoundAdapter::getRotation( TMat3& dstRotation )
    {
        auto _compoundRootBodyRef = m_compoundRef->root();
        if ( !_compoundRootBodyRef->adapter() )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundAdapter::getRotation() >>> the position of compound \"{0}\" \
                               can only be grabbed using the root's adapter getTransform() method", m_compoundRef->name() );
            return;
        }

        auto _compoundRootAdapterRef = dynamic_cast< TMjcCompoundBodyAdapter* >( _compoundRootBodyRef->adapter() );
        // grab the world-transform of the root-body (base is just a local-transform away from the root)
        auto _compoundRootBodyWorldTf = TMat4();
        _compoundRootAdapterRef->getTransform( _compoundRootBodyWorldTf );
        // compute the rotation of the base of the compound, using the root's transform
        dstRotation = ( _compoundRootBodyWorldTf * ( _compoundRootBodyRef->localTf() ).inverse() ).getRotation();
    }

    void TMjcCompoundAdapter::getTransform( TMat4& dstTransform )
    {
        auto _compoundRootBodyRef = m_compoundRef->root();
        if ( !_compoundRootBodyRef->adapter() )
        {
            TYSOC_CORE_ERROR( "TMjcCompoundAdapter::getTransform() >>> the position of compound \"{0}\" \
                               can only be grabbed using the root's adapter getTransform() method", m_compoundRef->name() );
            return;
        }

        auto _compoundRootAdapterRef = dynamic_cast< TMjcCompoundBodyAdapter* >( _compoundRootBodyRef->adapter() );
        // grab the world-transform of the root-body (base is just a local-transform away from the root)
        auto _compoundRootBodyWorldTf = TMat4();
        _compoundRootAdapterRef->getTransform( _compoundRootBodyWorldTf );
        // compute the transform of the base of the compound, using the root's transform
        dstTransform = _compoundRootBodyWorldTf * ( _compoundRootBodyRef->localTf() ).inverse();
    }

    void TMjcCompoundAdapter::onResourcesCreated()
    {
        assert( m_mjcModelRef );
        assert( m_mjcDataRef );

        // notify all bodies that the simulation has been initialized
        auto _compoundBodies = m_compoundRef->bodies();
        for ( auto _compoundBody : _compoundBodies )
        {
            if ( !_compoundBody )
            {
                TYSOC_CORE_WARN( "TMjcCompoundAdapter::onResourcesCreated() >>> there's a body in the \
                                  compound \"{0}\" that is null", m_compoundRef->name() );
                continue;
            }

            if ( !_compoundBody->adapter() )
            {
                TYSOC_CORE_WARN( "TMjcCompoundAdapter::onResourcesCreated() >>> body \"{0}\" hasn't got \
                                  an adapter that links it to the backend", _compoundBody->name() );
                continue;
            }

            auto _compoundBodyAdapter = dynamic_cast< TMjcCompoundBodyAdapter* >( _compoundBody->adapter() );
            _compoundBodyAdapter->setMjcModelRef( m_mjcModelRef );
            _compoundBodyAdapter->setMjcDataRef( m_mjcDataRef );
            _compoundBodyAdapter->onResourcesCreated();
        }
    }

}