#pragma once

#include <loco_common_mujoco.h>
#include <utils/loco_parsing_element.h>
#include <kinematic_trees/loco_kinematic_tree_collider_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeCollider;
}}

namespace loco {
namespace kintree {

    class TMujocoKinematicTreeColliderAdapter : public TIKinematicTreeColliderAdapter
    {
    public :

        TMujocoKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref );

        TMujocoKinematicTreeColliderAdapter( const TMujocoKinematicTreeColliderAdapter& other ) = delete;

        TMujocoKinematicTreeColliderAdapter& operator=( const TMujocoKinematicTreeColliderAdapter& other ) = delete;

        ~TMujocoKinematicTreeColliderAdapter();

        void Build() override;

        void Initialize() override;

        void SetLocalTransform( const TMat4& local_tf ) override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeCollisionGroup( int collision_group ) override;

        void ChangeCollisionMask( int collision_mask ) override;

        void ChangeFriction( const TScalar&  ) override;

        void SetMjcModel( mjModel* mj_model_ref ) { m_MjcModelRef = mj_model_ref; }

        void SetMjcData( mjData* mj_data_ref ) { m_MjcDataRef = mj_data_ref; }

        const parsing::TElement* element_resources() const { return m_MjcfElementResources.get(); }

        const parsing::TElement* element_asset_resources() const { return m_MjcfElementAssetResources.get(); }

        ssize_t mjc_geom_id() const { return m_MjcGeomId; }

        ssize_t mjc_geom_mesh_id() const { return m_MjcGeomMeshId; }

        double mjc_geom_rbound() const { return m_MjcGeomRbound; }

    private :

        void _ResizeMesh( const TVec3& new_size );

        void _ResizePrimitive( const TVec3& new_size );

    private :

        mjModel* m_MjcModelRef = nullptr;

        mjData* m_MjcDataRef = nullptr;

        ssize_t m_MjcGeomId = -1;

        ssize_t m_MjcGeomMeshId = -1;

        ssize_t m_MjcGeomMeshVertNum = -1;

        ssize_t m_MjcGeomMeshFaceNum = -1;

        ssize_t m_MjcGeomMeshVertStartAddr = -1;

        ssize_t m_MjcGeomMeshFaceStartAddr = -1;

        double m_MjcGeomRbound = 0.0;

        std::unique_ptr<parsing::TElement> m_MjcfElementResources = nullptr;

        std::unique_ptr<parsing::TElement> m_MjcfElementAssetResources = nullptr;

        TVec3 m_Size;

        TVec3 m_Size0;
    };
}}