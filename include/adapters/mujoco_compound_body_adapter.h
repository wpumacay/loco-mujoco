#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <compounds/compound_body.h>
#include <adapters/body_adapter.h>
#include <adapters/mujoco_collision_adapter.h>
#include <adapters/mujoco_compound_joint_adapter.h>

namespace tysoc {

    class TMjcCompoundBodyAdapter : public TIBodyAdapter
    {

    public :

        TMjcCompoundBodyAdapter( TCompoundBody* bodyRef );

        ~TMjcCompoundBodyAdapter();

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

        void setMjcModelRef( mjModel* mjcModelRef ) { m_mjcModelRef = mjcModelRef; }

        void setMjcDataRef( mjData* mjcDataRef ) { m_mjcDataRef = mjcDataRef; }

        mjcf::GenericElement* mjcfResource() const { return m_mjcfXmlResource; }

        mjcf::GenericElement* mjcfAssetResources() const { return m_mjcfXmlAssetResources; }

        int mjcBodyId() { return m_mjcBodyId; }

        void onResourcesCreated();

    private :

        void _updateTransformInBackend();
        void _updateWorldTransform_freeRoot( const TMat4& worldTransform );
        void _updateWorldTransform_fixedRoot( const TMat4& worldTransform );
        void _updateRelativeTransform( const TMat4& localTransform );

    private :

        /* Reference to mujoco-backend mjModel global struct (used to access model|structure related information) */
        mjModel* m_mjcModelRef;

        /* Reference to mujoco-backend mjData global struct (used to access kin-dyn elements used for computation) */
        mjData* m_mjcDataRef;

        /* Unique identifier of this body in the mujoco-backend */
        int m_mjcBodyId;

        // @todo: changes these for stacked-allocated structs (it seems there's no need to use pointers)

        /* Mjcf body xml-resource where place information required for the mujoco-backend to simulate this body  */
        mjcf::GenericElement* m_mjcfXmlResource;

        /* Mjcf assets xml-resource where place assets (files-refs, meshes-refs, ...) required for this body */
        mjcf::GenericElement* m_mjcfXmlAssetResources;

    };

    extern "C" TIBodyAdapter* simulation_createCompoundBodyAdapter( TCompoundBody* bodyRef );

}