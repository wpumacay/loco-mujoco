#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <adapters/joint_adapter.h>

namespace tysoc {

    class TMjcCompoundJointAdapter : public TIJointAdapter
    {

    public :

        TMjcCompoundJointAdapter( TJoint* jointRef );

        ~TMjcCompoundJointAdapter();

        void build() override;

        void reset() override;

        void preStep() override;

        void postStep() override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void setQpos( const std::array< TScalar, TYSOC_MAX_NUM_QPOS >& qpos ) override;

        void setQvel( const std::array< TScalar, TYSOC_MAX_NUM_QVEL >& qvel ) override;

        void getQpos( std::array< TScalar, TYSOC_MAX_NUM_QPOS >& dstQpos ) override;

        void getQvel( std::array< TScalar, TYSOC_MAX_NUM_QVEL >& dstQvel ) override;

        void changeLimits( const TVec2& limits ) override;

        void setMjcModelRef( mjModel* mjcModelRef ) { m_mjcModelRef = mjcModelRef; }

        void setMjcDataRef( mjData* mjcDataRef ) { m_mjcDataRef = mjcDataRef; }

        mjcf::GenericElement* mjcfResource() const { return m_mjcfXmlResource; }

        int mjcJointId() const { return m_mjcJointId; }

        void onResourcesCreated();

    private :

        mjModel* m_mjcModelRef;

        mjData* m_mjcDataRef;

        int m_mjcJointId;
        int m_mjcQposAdr;
        int m_mjcQvelAdr;
        int m_mjcQposNum;
        int m_mjcQvelNum;

        mjcf::GenericElement* m_mjcfXmlResource;

    };

    extern "C" TIJointAdapter* simulation_createCompoundJointAdapter( TJoint* jointRef );

}