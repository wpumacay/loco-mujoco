#pragma once

#include <loco_common_mujoco.h>
#include <utils/loco_parsing_element.h>
#include <primitives/loco_single_body_adapter.h>
#include <primitives/loco_single_body_collider_adapter_mujoco.h>
#include <primitives/loco_single_body_constraint_adapter_mujoco.h>

namespace loco {
    class TSingleBody;
}

namespace loco {
namespace mujoco {

    // Where the detached objects rest-grid starts
    const TVec3 DETACHED_REST_GRID_START = { 0.0, 0.0, 100.0 };
    // The amount in x-y-z in between elements in the rest-grid
    const TVec3 DETACHED_REST_GRID_DELTA = { 1.0, 1.0, 1.0 };
    // Size of the rest-grid for detached objects
    const ssize_t DETACHED_REST_GRID_SIZE = 10;
    const ssize_t DETACHED_REST_GRID_SIZE_POW2 = 100;

    class TMujocoSingleBodyAdapter : public TISingleBodyAdapter
    {
    public :

        TMujocoSingleBodyAdapter( TSingleBody* bodyRef );

        TMujocoSingleBodyAdapter( const TMujocoSingleBodyAdapter& other ) = delete;

        TMujocoSingleBodyAdapter& operator= ( const TMujocoSingleBodyAdapter& other ) = delete;

        ~TMujocoSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void OnDetach() override;

        void SetTransform( const TMat4& transform ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void SetForceCOM( const TVec3& force_com ) override;

        void SetTorqueCOM( const TVec3& torque_com ) override;

        void GetTransform( TMat4& dstTransform ) /* const */ override;

        void GetLinearVelocity( TVec3& dst_linear_vel ) /* const */ override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) /* const */ override;

        void SetMjcModel( mjModel* mjModelRef );

        void SetMjcData( mjData* mjDataRef );

        void HideMjcObject();

        parsing::TElement* element_resources() { return m_mjcfElementResources.get(); }

        const parsing::TElement* element_resources() const { return m_mjcfElementResources.get(); }

        parsing::TElement* element_asset_resources() { return m_mjcfElementAssetResources.get(); }

        const parsing::TElement* element_asset_resources() const { return m_mjcfElementAssetResources.get(); }

        mjModel* mjc_model() { return m_mjcModelRef; }

        const mjModel* mjc_model() const { return m_mjcModelRef; }

        mjData* mjc_data() { return m_mjcDataRef; }

        const mjData* mjc_data() const { return m_mjcDataRef; }

        ssize_t mjc_body_id() const { return m_mjcBodyId; }

        ssize_t mjc_joint_id() const { return m_mjcJointId; }

        ssize_t mjc_joint_qpos_num() const { return m_mjcJointQposNum; }

        ssize_t mjc_joint_qvel_num() const { return m_mjcJointQvelNum; }

        ssize_t mjc_joint_qpos_adr() const { return m_mjcJointQposAdr; }

        ssize_t mjc_joint_qvel_adr() const { return m_mjcJointQvelAdr; }

    private :

        mjModel* m_mjcModelRef;
        mjData* m_mjcDataRef;

        ssize_t m_mjcBodyId;
        ssize_t m_mjcJointId;
        ssize_t m_mjcJointQposAdr;
        ssize_t m_mjcJointQvelAdr;
        ssize_t m_mjcJointQposNum;
        ssize_t m_mjcJointQvelNum;
        ssize_t m_mjcGeomId;

        std::unique_ptr<parsing::TElement> m_mjcfElementResources;
        std::unique_ptr<parsing::TElement> m_mjcfElementAssetResources;

        TMat4 m_DetachedRestTransform;
        static ssize_t s_DetachedNum;
    };

}}