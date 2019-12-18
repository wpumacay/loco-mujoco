#pragma once

#include <adapters/compound_adapter.h>
#include <adapters/mujoco_collision_adapter.h>
#include <adapters/mujoco_compound_joint_adapter.h>
#include <adapters/mujoco_compound_body_adapter.h>

namespace tysoc
{

    class TMjcCompoundAdapter : public TICompoundAdapter
    {

    public :

        TMjcCompoundAdapter( TCompound* compoundRef );

        ~TMjcCompoundAdapter();

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

    public :

        void setMjcModelRef( mjModel* mjcModelRef ) { m_mjcModelRef = mjcModelRef; }

        void setMjcDataRef( mjData* mjcDataRef ) { m_mjcDataRef = mjcDataRef; }

        mjcf::GenericElement* mjcfResource() const { return m_mjcfXmlResource; }

        mjcf::GenericElement* mjcfAssetResources() const { return m_mjcfXmlAssetResources; }

        void onResourcesCreated();

    private :

        /* Reference to mujoco-backend mjModel global struct (used to access model|structure related information) */
        mjModel* m_mjcModelRef;

        /* Reference to mujoco-backend mjData global struct (used to access kin-dyn elements used for computation) */
        mjData* m_mjcDataRef;

        // @todo: changes these for stacked-allocated structs (it seems there's no need to use pointers)

        /* Mjcf body xml-resource where place information required for the mujoco-backend to simulate this body  */
        mjcf::GenericElement* m_mjcfXmlResource;

        /* Mjcf assets xml-resource where place assets (files-refs, meshes-refs, ...) required for this body */
        mjcf::GenericElement* m_mjcfXmlAssetResources;

    };

}