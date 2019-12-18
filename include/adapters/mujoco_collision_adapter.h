#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <adapters/collision_adapter.h>

#define COLLISION_DEFAULT_HFIELD_BASE 1.0f

namespace tysoc {

    class TCollision;

    class TMjcCollisionAdapter : public TICollisionAdapter
    {

    public :

        TMjcCollisionAdapter( TCollision* collisionPtr );

        ~TMjcCollisionAdapter();

        void build() override;

        void reset() override;

        void preStep() override;

        void postStep() override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void changeSize( const TVec3& newSize ) override;

        void changeElevationData( const std::vector< float >& heightData ) override;

        void setMjcModelRef( mjModel* mjcModelPtr ) { m_mjcModelPtr = mjcModelPtr; }

        void setMjcDataRef( mjData* mjcDataPtr ) { m_mjcDataPtr = mjcDataPtr; }


        mjcf::GenericElement* mjcfResource() { return m_mjcfXmlResource; }

        mjcf::GenericElement* mjcfAssetResource(){ return m_mjcfXmlAssetResource; }

        int mjcGeomId() { return m_mjcGeomId; }

        void onResourcesCreated();

    private :

        mjModel* m_mjcModelPtr;

        mjData* m_mjcDataPtr;

        int m_mjcGeomId;
        int m_mjcGeomMeshId;
        int m_mjcGeomHFieldId;
        int m_mjcGeomHFieldStartAddr;
        int m_mjcGeomHFieldNRows;
        int m_mjcGeomHFieldNCols;

        TScalar m_mjcRbound;

        mjcf::GenericElement* m_mjcfXmlResource;
        mjcf::GenericElement* m_mjcfXmlAssetResource;
    };

    extern "C" TICollisionAdapter* simulation_createCollisionAdapter( TCollision* collisionPtr );

}