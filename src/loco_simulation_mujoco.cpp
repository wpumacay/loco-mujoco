
#include <loco_simulation_mujoco.h>

namespace loco {
namespace mujoco {

    //// @sanitycheck
    //// How the simulation-creation process works (backend=MuJoCo)
    ////    * Within the constructor, we declare the resources (as default to nullptr) and create the adapters
    ////    * Within the _InitializeInternal method is where the bulk of the process happens :
    ////        > First we collect the mjcf-xml data from the adapters, created during their "Build" method.
    ////        > We then assemble these xml-data into a single xml-data object that represents the
    ////          simulation-model. We save this xml-data into disk to load it later using the MuJoCo-API.
    ////        > Finally, we load the simulation-model from the xml-data on disk using the MuJoCo-API,
    ////          creating the internal mujoco-simulation, and pass the handle to the mujoco-internals
    ////          (mjModel, mjData) to the adapters for their proper use.

    bool TMujocoSimulation::s_HasActivatedMujoco = false;

    TMujocoSimulation::TMujocoSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_backendId = "MUJOCO";

        m_mjcModel = nullptr;
        m_mjcData = nullptr;
        m_mjcfSimulationElement = nullptr;

        _CreateSingleBodyAdapters();
        //// _CreateCompoundAdapters();
        //// _CreateKintreeAdapters();
        //// _CreateTerrainGeneratorAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TMujocoSimulation @ {0}", tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TMujocoSimulation @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TMujocoSimulation::_CreateSingleBodyAdapters()
    {
        auto single_bodies = m_scenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TMujocoSingleBodyAdapter>( single_body );
            single_body->SetBodyAdapter( single_body_adapter.get() );
            m_singleBodyAdapters.push_back( std::move( single_body_adapter ) );
        }
    }

    TMujocoSimulation::~TMujocoSimulation()
    {
        m_mjcModel = nullptr;
        m_mjcData = nullptr;
        m_mjcfSimulationElement = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TMujocoSimulation @ {0}", tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TMujocoSimulation @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TMujocoSimulation::_InitializeInternal()
    {
        // Create empty mjcf-xml structure to store the simulation resources
        const std::string empty_mjcf_str =
        R"( <mujoco>
                <compiler inertiafromgeom="true" coordinate="local" angle="degree"/>
                <size njmax="10000" nconmax="40000"/>
                <asset>
                  <!-- place assets here -->
                </asset>

                <worldbody>
                  <!-- place bodies|compounds|kintrees here -->
                </worldbody>
            </mujoco> )";
        m_mjcfSimulationElement = parsing::TElement::CreateFromXmlString( parsing::eSchemaType::MJCF, empty_mjcf_str );
        m_mjcfAssetsNames = std::set<std::string>();
        m_mjcfAssetsFilepaths = std::set<std::string>();

        _CollectResourcesFromSingleBodies();
        // _CollectResourcesFromCompounds();
        // _CollectResourcesFromKintrees();
        // _CollectResourcesFromTerrainGenerators();

        // Store the xml-resources for this simulation into disk
        m_mjcfSimulationElement->SaveToXml( "simulation.xml" );

        if ( !TMujocoSimulation::s_HasActivatedMujoco )
        {
            mj_activate( loco::mujoco::LOCO_MUJOCO_LICENSE.c_str() );
            TMujocoSimulation::s_HasActivatedMujoco = true;
        }

        // Load the simulation from the xml-file created above *************************************
        const size_t error_buffer_size = 1000;
        char error_buffer[error_buffer_size];
        m_mjcModel = std::unique_ptr<mjModel, MjcModelDeleter>( mj_loadXML( "simulation.xml", nullptr, error_buffer, error_buffer_size ) );
        if ( !m_mjcModel )
        {
            LOCO_CORE_ERROR( "TMujocoSimulation::_InitializeInternal >>> Couldn't initialize mujoco-API" );
            LOCO_CORE_ERROR( "\tError-message   : {0}", error_buffer );
            return false;
        }
        m_mjcData = std::unique_ptr<mjData, MjcDataDeleter>( mj_makeData( m_mjcModel.get() ) );
        //******************************************************************************************

        for ( auto& single_body_adapter : m_singleBodyAdapters )
        {
            if ( auto mjc_adapter = dynamic_cast<TMujocoSingleBodyAdapter*>( single_body_adapter.get() ) )
            {
                mjc_adapter->SetMjcModel( m_mjcModel.get() );
                mjc_adapter->SetMjcData( m_mjcData.get() );
            }
        }

        // Take a single step of kinematics computation (to put everything in place)
        mj_kinematics( m_mjcModel.get(), m_mjcData.get() );

        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nq: {0}", m_mjcModel->nq );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nv: {0}", m_mjcModel->nv );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nu: {0}", m_mjcModel->nu );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nbody: {0}", m_mjcModel->nbody );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-njnt: {0}", m_mjcModel->njnt );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-ngeom: {0}", m_mjcModel->ngeom );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nsensor: {0}", m_mjcModel->nsensor );

        return true;
    }

    void TMujocoSimulation::_CollectResourcesFromSingleBodies()
    {
        LOCO_CORE_ASSERT( m_mjcfSimulationElement, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                          there is no mjcf-simulation-element to place the resources in" );

        auto world_body_element = m_mjcfSimulationElement->GetFirstChildOfType( LOCO_MJCF_WORLDBODY_TAG );
        auto assets_element = m_mjcfSimulationElement->GetFirstChildOfType( LOCO_MJCF_ASSET_TAG );

        LOCO_CORE_ASSERT( world_body_element, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                          there is no world-body element in the mjcf-simulation-element" );
        LOCO_CORE_ASSERT( assets_element, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                          there is no asset element in the mjcf-simulation-element" );

        for ( auto& single_body_adapter : m_singleBodyAdapters )
        {
            if ( !single_body_adapter )
            {
                LOCO_CORE_ERROR( "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> there's a \
                                  rogue nullptr single-body adapter that was added to the adapters list" );
                continue;
            }

            auto mjc_adapter = static_cast<TMujocoSingleBodyAdapter*>( single_body_adapter.get() );
            auto mjcf_element = mjc_adapter->element_resources();
            LOCO_CORE_ASSERT( mjcf_element, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                              single-body mjc-adapter must have a mjcf-element with its resources on it (got nullptr instead)" );
            world_body_element->Add( parsing::TElement::CloneElement( mjcf_element ) );

            if ( auto mjcf_asset_element = mjc_adapter->element_asset_resources() )
            {
                for ( size_t i = 0; i < mjcf_asset_element->num_children(); i++ )
                {
                    auto asset_element = mjcf_asset_element->get_child( i );
                    const std::string asset_type = asset_element->elementType();
                    if ( asset_type == LOCO_MJCF_MESH_TAG )
                    {
                        // Check that meshes that have both id and filepath equal are not duplicated,
                        // and those that only have the same id but different filepaths should have
                        // their medh-ids changed appropriately to distinguish them during loading
                        bool mesh_id_already_cached = false;
                        bool mesh_file_already_cached = false;
                        LOCO_CORE_ASSERT( asset_element->HasAttributeString( "name" ), "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                                          mesh-assets must have a valid mesh-id (but none found on mjcf element" );
                        const std::string mesh_id = asset_element->GetString( "name" );
                        if ( m_mjcfAssetsNames.find( mesh_id ) != m_mjcfAssetsNames.end() )
                            mesh_id_already_cached = true;
                        else
                            m_mjcfAssetsNames.emplace( mesh_id );
                        LOCO_CORE_ASSERT( asset_element->HasAttributeString( "file" ), "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                                          mesh-assets must have a valid mesh-file (but none found on mjcf element" );
                        const std::string mesh_file = asset_element->GetString( "file" );
                        if ( m_mjcfAssetsFilepaths.find( mesh_file ) != m_mjcfAssetsFilepaths.end() )
                            mesh_file_already_cached = true;
                        else
                            m_mjcfAssetsFilepaths.emplace( mesh_file );

                        if ( !mesh_id_already_cached )
                        {
                            // Can add mesh-asset normally, as there are no id-duplicates
                            assets_element->Add( parsing::TElement::CloneElement( asset_element ) );
                        }
                        else if ( !mesh_file_already_cached )
                        {
                            // Mesh-asset has same id, but different filepath (so it's a different resource)
                            static ssize_t num_duplicates = 1;
                            const std::string new_mesh_id = mesh_id + "_" + std::to_string( num_duplicates++ );
                            auto added_mjcf_body = world_body_element->get_child( world_body_element->num_children() - 1 );
                            auto added_mjcf_collider = added_mjcf_body->GetFirstChildOfType( LOCO_MJCF_GEOM_TAG );
                            LOCO_CORE_ASSERT( added_mjcf_collider, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                                              added mesh-body must have a valid mesh-collider-geom (but none found on mjcf element)" );
                            added_mjcf_collider->SetString( "mesh", new_mesh_id );
                            asset_element->SetString( "name", new_mesh_id );
                            // Can add modified mesh-asset, with mesh-id modified to avoid duplicates
                            assets_element->Add( parsing::TElement::CloneElement( asset_element ) );
                        }
                    }
                    else if ( asset_type == LOCO_MJCF_HFIELD_TAG )
                    {
                        const std::string hfield_id = asset_element->GetString( "name" );
                        if ( m_mjcfAssetsNames.find( hfield_id ) != m_mjcfAssetsNames.end() )
                            continue; // hfield-id already cached
                        else
                            m_mjcfAssetsNames.emplace( hfield_id );
                        assets_element->Add( parsing::TElement::CloneElement( asset_element ) );
                    }
                    else
                    {
                        LOCO_CORE_ERROR( "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                                          asset-type {0} not supported", asset_type );
                        continue;
                    }
                }
            }
        }
    }

    void TMujocoSimulation::_PreStepInternal()
    {
        // Make sure recycled objects are not being simulated
        for ( ssize_t i = 0; i < m_singleBodyAdaptersRecycled.size(); i++ )
            if ( auto mjc_single_body_adapter = dynamic_cast<TMujocoSingleBodyAdapter*>( m_singleBodyAdaptersRecycled[i].get() ) )
                mjc_single_body_adapter->HideMjcObject();
    }

    void TMujocoSimulation::_SimStepInternal()
    {
        if ( !m_mjcModel )
        {
            LOCO_CORE_WARN( "TMujocoSimulation::_SimStepInternal >>> mjModel structure is required \
                             for taking a simulation step" );
            return;
        }

        if ( !m_mjcData )
        {
            LOCO_CORE_WARN( "TMujocoSimulation::_SimStepInternal >>> mjData structure is required \
                             for taking a simulation step" );
            return;
        }

        const mjtNum target_fps = 1.0 / 60.0;
        const mjtNum sim_start = m_mjcData->time;
        while ( ( m_mjcData->time - sim_start ) < target_fps )
            mj_step( m_mjcModel.get(), m_mjcData.get() );
    }

    void TMujocoSimulation::_PostStepInternal()
    {
        // Do nothing here, as call to adapters is enough (made in base)
    }

    void TMujocoSimulation::_ResetInternal()
    {
        // Do nothing here, as call to adapters is enough (made in base)
    }

extern "C" TISimulation* simulation_create( loco::TScenario* scenarioRef )
{
    return new loco::mujoco::TMujocoSimulation( scenarioRef );
}

extern "C" TISingleBodyAdapter* single_body_create( loco::TSingleBody* single_body_ref )
{
    // if recycled available, return recycled
    // else, return nullptr (mujoco can't create dynamically yet)
    return nullptr;
}

}}