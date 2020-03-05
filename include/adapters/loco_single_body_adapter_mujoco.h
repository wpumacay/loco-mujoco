#pragma once

#include <loco_common_mujoco.h>
#include <utils/loco_parsing_element.h>
#include <adapters/loco_body_adapter.h>
#include <adapters/loco_collision_adapter_mujoco.h>

namespace loco {
    class TIBody;
}

namespace loco {
namespace mujoco {

    class TMujocoSingleBodyAdapter : public TIBodyAdapter
    {
    public :

        TMujocoSingleBodyAdapter( TIBody* bodyRef );

        TMujocoSingleBodyAdapter( const TMujocoSingleBodyAdapter& other ) = delete;

        TMujocoSingleBodyAdapter& operator= ( const TMujocoSingleBodyAdapter& other ) = delete;

        ~TMujocoSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void PreStep() override;

        void PostStep() override;

        void SetPosition( const TVec3& position ) override;

        void SetRotation( const TMat3& rotation ) override;

        void SetTransform( const TMat4& transform ) override;

        void GetPosition( TVec3& dstPosition ) /* const */ override;

        void GetRotation( TMat3& dstRotation ) /* const */ override;

        void GetTransform( TMat4& dstTransform ) /* const */ override;

        void SetLocalPosition( const TVec3& position ) override;

        void SetLocalRotation( const TMat3& rotation ) override;

        void SetLocalTransform( const TMat4& transform ) override;

        void GetLocalPosition( TVec3& dstPosition ) override;

        void GetLocalRotation( TMat3& dstRotation ) override;

        void GetLocalTransform( TMat4& dstTransform ) override;

        void SetMjcModel( mjModel* mjModelRef ) { m_mjcModelRef = mjModelRef; }

        void SetMjcData( mjData* mjDataRef ) { m_mjcDataRef = mjDataRef; }

        const parsing::TElement* element_resources() const { return m_mjcfElementResources.get(); }

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

        std::unique_ptr<parsing::TElement> m_mjcfElementResources;
        std::unique_ptr<parsing::TElement> m_mjcfElementAssetResources;
    };

}}