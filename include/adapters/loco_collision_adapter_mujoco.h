#pragma once

#include <loco_common_mujoco.h>
#include <adapters/loco_collision_adapter.h>
#include <utils/loco_parsing_element.h>

namespace loco {
    class TCollision;
}

namespace loco {
namespace mujoco {

    const float LOCO_MUJOCO_HFIELD_BASE = 1.0f;

    class TMujocoCollisionAdapter : public TICollisionAdapter
    {
    public :

        TMujocoCollisionAdapter( TCollision* collisionRef );

        ~TMujocoCollisionAdapter();

        void Build() override;

        void Initialize() override;

        void PreStep() override;

        void PostStep() override;

        void Reset() override;

        void SetLocalPosition( const TVec3& position ) override;

        void SetLocalRotation( const TMat3& rotation ) override;

        void SetLocalTransform( const TMat4& transform ) override;

        void ChangeSize( const TVec3& newSize ) override;

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collisionGroup ) override;

        void ChangeCollisionMask( int collisionMask ) override;

        void SetMjcModel( mjModel* mjModelRef ) { m_mjcModelRef = mjModelRef; }

        void SetMjcData( mjData* mjDataRef ) { m_mjcDataRef = mjDataRef; }

        const parsing::TElement* element_resources() const { return m_mjcfElementResources.get(); }

        const parsing::TElement* element_asset_resources() const { return m_mjcfElementAssetResources.get(); }

        ssize_t mjc_geom_id() const { return m_mjcGeomId; }

        ssize_t mjc_geom_mesh_id() const { return m_mjcGeomMeshId; }

        ssize_t mjc_geom_hfield_id() const { return m_mjcGeomHFieldId; }

        ssize_t mjc_geom_hfield_start_addr() const { return m_mjcGeomHFieldStartAddr; }

        ssize_t mjc_geom_hfield_nrows() const { return m_mjcGeomHFieldNRows; }

        ssize_t mjc_geom_hfield_ncols() const { return m_mjcGeomHFieldNCols; }

        double mjc_geom_radius_bound() const { return m_mjcGeomRbound; }

    private :

        void _resize_mesh( const TVec3& new_size );

        void _resize_hfield( const TVec3& new_size );

        void _resize_primitive( const TVec3& new_size );

    private :

        mjModel* m_mjcModelRef;
        mjData* m_mjcDataRef;

        ssize_t m_mjcGeomId;
        ssize_t m_mjcGeomMeshId;
        ssize_t m_mjcGeomMeshVertNum;
        ssize_t m_mjcGeomMeshFaceNum;
        ssize_t m_mjcGeomMeshVertStartAddr;
        ssize_t m_mjcGeomMeshFaceStartAddr;
        ssize_t m_mjcGeomHFieldId;
        ssize_t m_mjcGeomHFieldStartAddr;
        ssize_t m_mjcGeomHFieldNRows;
        ssize_t m_mjcGeomHFieldNCols;
        double m_mjcGeomRbound;

        TVec3 m_size;
        TVec3 m_size0;

        std::unique_ptr<parsing::TElement> m_mjcfElementResources;
        std::unique_ptr<parsing::TElement> m_mjcfElementAssetResources;
    };

}}