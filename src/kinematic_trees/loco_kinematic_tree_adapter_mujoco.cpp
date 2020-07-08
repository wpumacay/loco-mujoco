
#include <kinematic_trees/loco_kinematic_tree_adapter_mujoco.h>

namespace loco {
namespace kintree {

    TMujocoKinematicTreeAdapter::TMujocoKinematicTreeAdapter( TKinematicTree* kintree_ref )
        : TIKinematicTreeAdapter( kintree_ref ) {}

    TMujocoKinematicTreeAdapter::~TMujocoKinematicTreeAdapter()
    {
        m_MjcModelRef = nullptr;
        m_MjcDataRef = nullptr;
        m_MjcfElementResources = nullptr;
        m_MjcfElementAssetsResources = nullptr;
    }

    void TMujocoKinematicTreeAdapter::Build()
    {
        m_MjcfElementResources = std::make_unique<parsing::TElement>( mujoco::LOCO_MJCF_WORLDBODY_TAG, parsing::eSchemaType::MJCF );
        auto root_body = m_KintreeRef->root();
        std::stack<std::pair<TKinematicTreeBody*, parsing::TElement*>> dfs_body_parentElm;
        dfs_body_parentElm.push( { root_body, m_MjcfElementResources.get() } );
        while ( !dfs_body_parentElm.empty() )
        {
            auto body_parentElm_pair = dfs_body_parentElm.top();
            dfs_body_parentElm.pop();
            auto curr_body = body_parentElm_pair.first;
            auto curr_parent_elm = body_parentElm_pair.second;
            if ( !curr_body || !curr_parent_elm )
            {
                LOCO_CORE_ERROR( "TMujocoKinematicTreeAdapter::Build >>> found nullptr, either body or "
                                 "parsing-resource. Error found while processing kintree {0}", m_KintreeRef->name() );
                continue;
            }

            auto mjc_body_adapter = std::make_unique<TMujocoKinematicTreeBodyAdapter>( curr_body );
            curr_body->SetBodyAdapter( mjc_body_adapter.get() );
            m_BodyAdapters.push_back( std::move( mjc_body_adapter ) );
            m_BodyAdapters.back()->Build();

            if ( auto body_element_resources = static_cast<TMujocoKinematicTreeBodyAdapter*>(
                                                m_BodyAdapters.back().get() )->element_resources() )
            {
                curr_parent_elm->Add( parsing::TElement::CloneElement( body_element_resources ) );
            }
            if ( auto body_element_assets_resources = static_cast<TMujocoKinematicTreeBodyAdapter*>(
                                                        m_BodyAdapters.back().get() )->element_assets_resources() )
            {
                if ( !m_MjcfElementAssetsResources )
                    m_MjcfElementAssetsResources = std::make_unique<parsing::TElement>( 
                                                            mujoco::LOCO_MJCF_ASSET_TAG, parsing::eSchemaType::MJCF );
                for ( ssize_t i = 0; i < body_element_assets_resources->num_children(); i++ )
                    m_MjcfElementAssetsResources->Add( parsing::TElement::CloneElement( 
                                                            body_element_assets_resources->get_child( i ) ) );
            }

            auto children = curr_body->children();
            for ( auto child : children )
                dfs_body_parentElm.push( { child, curr_parent_elm->get_child( curr_parent_elm->num_children() - 1 ) } );
        }
    }

    void TMujocoKinematicTreeAdapter::Initialize()
    {

    }

    void TMujocoKinematicTreeAdapter::Reset()
    {

    }

    void TMujocoKinematicTreeAdapter::SetTransform( const TMat4& tf )
    {

    }

    void TMujocoKinematicTreeAdapter::SetLinearVelocity( const TVec3& linear_vel )
    {

    }

    void TMujocoKinematicTreeAdapter::SetAngularVelocity( const TVec3& angular_vel )
    {

    }

    void TMujocoKinematicTreeAdapter::GetTransform( TMat4& dst_transform )
    {

    }

    void TMujocoKinematicTreeAdapter::GetLinearVelocity( TVec3& dst_linear_vel )
    {

    }

    void TMujocoKinematicTreeAdapter::GetAngularVelocity( TVec3& dst_angular_vel )
    {

    }

    void TMujocoKinematicTreeAdapter::SetMjcModel( mjModel* mj_model_ref )
    {
        m_MjcModelRef = mj_model_ref;
        for ( auto& body_adapter : m_BodyAdapters )
            if ( auto mjc_body_adapter = dynamic_cast<TMujocoKinematicTreeBodyAdapter*>( body_adapter.get() ) )
                mjc_body_adapter->SetMjcModel( mj_model_ref );
    }

    void TMujocoKinematicTreeAdapter::SetMjcData( mjData* mj_data_ref )
    {
        m_MjcDataRef = mj_data_ref;
        for ( auto& body_adapter : m_BodyAdapters )
            if ( auto mjc_body_adapter = dynamic_cast<TMujocoKinematicTreeBodyAdapter*>( body_adapter.get() ) )
                mjc_body_adapter->SetMjcData( mj_data_ref );
    }

}}