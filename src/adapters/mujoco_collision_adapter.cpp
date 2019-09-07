
#include <adapters/mujoco_collision_adapter.h>

namespace tysoc {

    TMjcCollisionAdapter::TMjcCollisionAdapter( TCollision* collisionPtr )
        : TICollisionAdapter( collisionPtr )
    {
        m_mjcModelPtr       = NULL;
        m_mjcDataPtr        = NULL;
        m_mjcfXmlResource   = NULL;
        m_mjcGeomId         = -1;
    }

    TMjcCollisionAdapter::~TMjcCollisionAdapter()
    {
        if ( m_mjcfXmlResource )
            delete m_mjcfXmlResource;

        m_mjcGeomId  = -1;

        m_mjcModelPtr = NULL;
        m_mjcDataPtr = NULL;
        m_mjcfXmlResource = NULL;
    }

    void TMjcCollisionAdapter::build()
    {
        assert( m_collisionPtr );

        m_mjcfXmlResource = new mjcf::GenericElement( "geom" );
        m_mjcfXmlResource->setAttributeString( "name", m_collisionPtr->name() );
        m_mjcfXmlResource->setAttributeVec3( "pos", m_collisionPtr->localPos() );
        m_mjcfXmlResource->setAttributeVec4( "quat", m_collisionPtr->localQuat() );
        m_mjcfXmlResource->setAttributeString( "type", mujoco::shapeType2MjcfShapeType( m_collisionPtr->shape() ) );
        m_mjcfXmlResource->setAttributeVec3( "size", mujoco::size2MjcfSize( m_collisionPtr->shape(), m_collisionPtr->size() ) );

        // @TODO: Tests these  thoroughly before exposing them for usage
        // m_mjcfXmlResource->setAttributeFloat( "density", m_collisionPtr->data().density );
        // m_mjcfXmlResource->setAttributeVec3( "friction", m_collisionPtr->data().friction );
        // m_mjcfXmlResource->setAttributeInt( "contype", m_collisionPtr->data().collisionGroup );
        // m_mjcfXmlResource->setAttributeInt( "conaffinity", m_collisionPtr->data().collisionMask );
    }

    void TMjcCollisionAdapter::reset()
    {
        // nothing required for now, as the functionality exposed in the other methods seems enough
    }

    void TMjcCollisionAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TMjcCollisionAdapter::setLocalPosition( const TVec3& position )
    {
        assert( m_collisionPtr );
        assert( m_mjcGeomId != -1 );

        m_mjcModelPtr->geom_pos[3 * m_mjcGeomId + 0] = position.x;
        m_mjcModelPtr->geom_pos[3 * m_mjcGeomId + 1] = position.y;
        m_mjcModelPtr->geom_pos[3 * m_mjcGeomId + 2] = position.z;
    }

    void TMjcCollisionAdapter::setLocalRotation( const TMat3& rotation )
    {
        assert( m_collisionPtr );
        assert( m_mjcGeomId != -1 );

        auto _quaternion = TMat3::toQuaternion( rotation );

        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 0] = _quaternion.w;
        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 1] = _quaternion.x;
        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 2] = _quaternion.y;
        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 3] = _quaternion.z;
    }

    void TMjcCollisionAdapter::setLocalTransform( const TMat4& transform )
    {
        auto _position = transform.getPosition();
        auto _quaternion = transform.getRotQuaternion();

        m_mjcModelPtr->geom_pos[3 * m_mjcGeomId + 0] = _position.x;
        m_mjcModelPtr->geom_pos[3 * m_mjcGeomId + 1] = _position.y;
        m_mjcModelPtr->geom_pos[3 * m_mjcGeomId + 2] = _position.z;

        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 0] = _quaternion.w;
        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 1] = _quaternion.x;
        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 2] = _quaternion.y;
        m_mjcModelPtr->geom_quat[4 * m_mjcGeomId + 3] = _quaternion.z;
    }

    void TMjcCollisionAdapter::changeSize( const TVec3& newSize )
    {
        assert( m_collisionPtr );
        assert( m_mjcGeomId != -1 );

        auto _newSizeMjcf = mujoco::size2MjcfSize( m_collisionPtr->shape(), newSize );
        auto _newRboundMjcf = mujoco::computeRbound( m_collisionPtr->shape(), newSize );

        m_mjcModelPtr->geom_size[3 * m_mjcGeomId + 0] = _newSizeMjcf.x;
        m_mjcModelPtr->geom_size[3 * m_mjcGeomId + 1] = _newSizeMjcf.y;
        m_mjcModelPtr->geom_size[3 * m_mjcGeomId + 2] = _newSizeMjcf.z;

        m_mjcModelPtr->geom_rbound[m_mjcGeomId] = _newRboundMjcf;
    }

    void TMjcCollisionAdapter::onResourcesCreated()
    {
        assert( m_mjcModelPtr );
        assert( m_mjcDataPtr );

        if ( m_mjcGeomId == -1 )
        {
            std::cout << "ERROR> mjc-collision-adapter should have been assigned a mjc-id by now" << std::endl;
            return;
        }

        m_mjcRbound = m_mjcModelPtr->geom_rbound[m_mjcGeomId];

        // @TODO: Add special functionality to handle HFIELDS and MESHES
    }

    extern "C" TICollisionAdapter* simulation_createCollisionAdapter( TCollision* collisionPtr )
    {
        return new TMjcCollisionAdapter( collisionPtr );
    }

}