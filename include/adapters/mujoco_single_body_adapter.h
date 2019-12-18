#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <primitives/single_body.h>
#include <adapters/body_adapter.h>
#include <adapters/mujoco_collision_adapter.h>

namespace tysoc {

    class TMjcSingleBodyAdapter : public TIBodyAdapter
    {

    public :

        TMjcSingleBodyAdapter( TSingleBody* bodyRef );

        ~TMjcSingleBodyAdapter();

        void build() override;

        void reset() override;

        void preStep() override;

        void postStep() override;

        void setPosition( const TVec3& position ) override;

        void setRotation( const TMat3& rotation ) override;

        void setTransform( const TMat4& transform ) override;

        void getPosition( TVec3& dstPosition ) override;

        void getRotation( TMat3& dstRotation ) override;

        void getTransform( TMat4& dstTransform ) override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void getLocalPosition( TVec3& dstPosition ) override;

        void getLocalRotation( TMat3& dstRotation ) override;

        void getLocalTransform( TMat4& dstTransform ) override;

        void setMjcModelRef( mjModel* mjcModelPtr ) { m_mjcModelPtr = mjcModelPtr; }

        void setMjcDataRef( mjData* mjcDataPtr ) { m_mjcDataPtr = mjcDataPtr; }

        mjcf::GenericElement* mjcfResource() const { return m_mjcfXmlResource; }

        mjcf::GenericElement* mjcfAssetResources() const { return m_mjcfXmlAssetResources; }

        int mjcBodyId() { return m_mjcBodyId; }

        void onResourcesCreated();

    private :

        mjModel* m_mjcModelPtr;

        mjData* m_mjcDataPtr;

        int m_mjcBodyId;
        int m_mjcJointId;
        int m_mjcQposAdr;
        int m_mjcQvelAdr;
        int m_mjcQposNum;
        int m_mjcQvelNum;

        mjcf::GenericElement* m_mjcfXmlResource;
        mjcf::GenericElement* m_mjcfXmlAssetResources;

    };

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TSingleBody* bodyRef );

}