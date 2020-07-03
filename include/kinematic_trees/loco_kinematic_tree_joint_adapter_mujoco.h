#pragma once

#include <loco_common_mujoco.h>
#include <utils/loco_parsing_element.h>
#include <kinematic_trees/loco_kinematic_tree_joint_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeJoint;
}}

namespace loco {
namespace kintree {

    class TMujocoKinematicTreeJointAdapter : public TIKinematicTreeJointAdapter
    {
    public :

        TMujocoKinematicTreeJointAdapter( TKinematicTreeJoint* joint_ref );

        TMujocoKinematicTreeJointAdapter( const TMujocoKinematicTreeJointAdapter& other ) = delete;

        TMujocoKinematicTreeJointAdapter& operator=( const TMujocoKinematicTreeJointAdapter& other ) = delete;

        ~TMujocoKinematicTreeJointAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetQpos( const std::vector<TScalar>& qpos ) override;

        void SetQvel( const std::vector<TScalar>& qvel ) override;

        void SetLocalTransform( const TMat4& local_tf ) override;

        void ChangeStiffness( const TScalar& stiffness ) override;

        void ChangeArmature( const TScalar& armature ) override;

        void ChangeDamping( const TScalar& damping ) override;

        void ChangeAxis( const TVec3& axis ) override;

        void ChangeLimits( const TVec2& limits ) override;

        void GetQpos( std::vector<TScalar>& dst_qpos ) override;

        void GetQvel( std::vector<TScalar>& dst_qvel ) override;

        void SetMjcModel( mjModel* mj_model_ref ) { m_MjcModelRef = mj_model_ref; }

        void SetMjcData( mjData* mj_data_ref ) { m_MjcDataRef = mj_data_ref; }

        std::vector<parsing::TElement*> elements_resources();

        std::vector<const parsing::TElement*> elements_resources() const;

        ssize_t mjc_joint_id() const { return m_MjcJointId; }

        ssize_t mjc_dof_id() const { return m_MjcDofId; }

        ssize_t mjc_joint_qposadr() const { return m_MjcJointQposAdr; }

        ssize_t mjc_joint_qveladr() const { return m_MjcJointQvelAdr; }

    private :

        mjModel* m_MjcModelRef = nullptr;

        mjData* m_MjcDataRef = nullptr;

        ssize_t m_MjcJointId = -1;

        ssize_t m_MjcDofId = -1;

        ssize_t m_MjcJointQposAdr = -1;

        ssize_t m_MjcJointQvelAdr = -1;

        std::vector<std::unique_ptr<parsing::TElement>> m_MjcfElementsResources;
    };
}}