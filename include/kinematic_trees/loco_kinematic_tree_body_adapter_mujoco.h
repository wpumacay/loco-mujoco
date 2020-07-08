#pragma once

#include <loco_common_mujoco.h>
#include <utils/loco_parsing_element.h>
#include <kinematic_trees/loco_kinematic_tree_body_adapter.h>
#include <kinematic_trees/loco_kinematic_tree_collider_adapter_mujoco.h>
#include <kinematic_trees/loco_kinematic_tree_joint_adapter_mujoco.h>

namespace loco {
namespace kintree {
    class TKinematicTreeBody;
    class TIKinematicTreeColliderAdapter;
    class TIKinematicTreeJointAdapter;
}}

namespace loco {
namespace kintree {

    class TMujocoKinematicTreeBodyAdapter : public TIKinematicTreeBodyAdapter
    {
    public :

        TMujocoKinematicTreeBodyAdapter( TKinematicTreeBody* body_ref );

        TMujocoKinematicTreeBodyAdapter( const TMujocoKinematicTreeBodyAdapter& other ) = delete;

        TMujocoKinematicTreeBodyAdapter& operator=( const TMujocoKinematicTreeBodyAdapter& other ) = delete;

        ~TMujocoKinematicTreeBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetForceCOM( const TVec3& force ) override;

        void SetTorqueCOM( const TVec3& torque ) override;

        void GetTransform( TMat4& dst_transform ) override;

        void SetMjcModel( mjModel* mj_model_ref );

        void SetMjcData( mjData* mj_data_ref );

        ssize_t mjc_body_id() const { return m_MjcBodyId; }

        parsing::TElement* element_resources() { return m_MjcfElementResources.get(); }

        const parsing::TElement* element_resources() const { return m_MjcfElementResources.get(); }

        parsing::TElement* element_assets_resources() { return m_MjcfElementAssetsResources.get(); }

        const parsing::TElement* element_assets_resources() const { return m_MjcfElementAssetsResources.get(); }

    private :

        mjModel* m_MjcModelRef = nullptr;

        mjData* m_MjcDataRef = nullptr;

        ssize_t m_MjcBodyId = -1;

        std::unique_ptr<parsing::TElement> m_MjcfElementResources = nullptr;

        std::unique_ptr<parsing::TElement> m_MjcfElementAssetsResources = nullptr;
    };
}}