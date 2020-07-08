#pragma once

#include <utils/loco_parsing_element.h>
#include <kinematic_trees/loco_kinematic_tree_adapter.h>
#include <kinematic_trees/loco_kinematic_tree_body_adapter_mujoco.h>

namespace loco {
namespace kintree {
    class TKinematicTree;
    class TIKinematicTreeBodyAdapter;
}}

namespace loco {
namespace kintree {

    class TMujocoKinematicTreeAdapter : public TIKinematicTreeAdapter
    {
    public :

        TMujocoKinematicTreeAdapter( TKinematicTree* kintree_ref );

        TMujocoKinematicTreeAdapter( const TMujocoKinematicTreeAdapter& other ) = delete;

        TMujocoKinematicTreeAdapter& operator=( const TMujocoKinematicTreeAdapter& other ) = delete;

        ~TMujocoKinematicTreeAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetTransform( const TMat4& tf ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void GetTransform( TMat4& dst_transform ) override;

        void GetLinearVelocity( TVec3& dst_linear_vel ) override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) override;

        void SetMjcModel( mjModel* mj_model_ref );

        void SetMjcData( mjData* mj_data_ref );

    private :

        mjModel* m_MjcModelRef = nullptr;

        mjData* m_MjcDataRef = nullptr;

        std::unique_ptr<parsing::TElement> m_MjcfElementResources = nullptr;

        std::unique_ptr<parsing::TElement> m_MjcfElementAssetsResources = nullptr;
    };
}}