#pragma once

#include <mujoco_common.h>
#include <mujoco_utils.h>

#include <adapters/body_adapter.h>

namespace tysoc {

    class TMjcBodyAdapter : public TIBodyAdapter
    {

    public :

        TMjcBodyAdapter( TBody* bodyPtr );

        ~TMjcBodyAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setPosition( const TVec3& position ) override;

        void setRotation( const TMat3& rotation ) override;

        void setTransform( const TMat4& transform ) override;

        void getPosition( TVec3& dstPosition ) override;

        void getRotation( TMat3& dstRotation ) override;

        void getTransform( TMat4& dstTransform ) override;

        void setMjcModel( mjModel* mjcModelPtr ) { m_mjcModelPtr = mjcModelPtr; }

        void setMjcData( mjData* mjcDataPtr ) { m_mjcDataPtr = mjcDataPtr; }

        void setMjcBodyId( int mjcBodyId ) { m_mjcBodyId = mjcBodyId; }

        mjcf::GenericElement* mjcfResource() { return m_mjcfXmlResource; }

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

    };

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr );

}