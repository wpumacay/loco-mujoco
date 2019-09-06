#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <adapters/collision_adapter.h>

namespace tysoc {

    class TCollision;

    class TMjcCollisionAdapter : public TICollisionAdapter
    {

    public :

        TMjcCollisionAdapter( TCollision* collisionPtr );

        ~TMjcCollisionAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void changeSize( const TVec3& newSize ) override;

        void setMjcModel( mjModel* mjcModelPtr ) { m_mjcModelPtr = mjcModelPtr; }

        void setMjcData( mjData* mjcDataPtr ) { m_mjcDataPtr = mjcDataPtr; }

        void setMjcGeomId( int mjcGeomId ) { m_mjcGeomId = mjcGeomId; }

        mjcf::GenericElement* mjcfResource() { return m_mjcfXmlResource; }

        int mjcGeomId() { return m_mjcGeomId; }

        void onResourcesCreated();

    private :

        mjModel* m_mjcModelPtr;

        mjData* m_mjcDataPtr;

        int m_mjcGeomId;

        TScalar m_mjcRbound;

        mjcf::GenericElement* m_mjcfXmlResource;

    };

    extern "C" TICollisionAdapter* simulation_createCollisionAdapter( TCollision* collisionPtr );

}