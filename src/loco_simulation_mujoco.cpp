
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
        m_BackendId = "MUJOCO";

        m_MjcModel = nullptr;
        m_MjcData = nullptr;
        m_MjcfSimulationElement = nullptr;

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
        auto single_bodies = m_ScenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TMujocoSingleBodyAdapter>( single_body );
            single_body->SetBodyAdapter( single_body_adapter.get() );
            m_SingleBodyAdapters.push_back( std::move( single_body_adapter ) );
        }
    }

    TMujocoSimulation::~TMujocoSimulation()
    {
        m_MjcModel = nullptr;
        m_MjcData = nullptr;
        m_MjcfSimulationElement = nullptr;

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
                <option timestep="0.002" gravity="0 0 -9.81"/>
                <asset>
                  <!-- place assets here -->
                </asset>

                <worldbody>
                  <!-- place bodies|compounds|kintrees here -->
                </worldbody>
            </mujoco> )";
        m_MjcfSimulationElement = parsing::TElement::CreateFromXmlString( parsing::eSchemaType::MJCF, empty_mjcf_str );
        m_MjcfAssetsNames = std::set<std::string>();
        m_MjcfAssetsFilepaths = std::set<std::string>();

        // Set extra options for the simulation (internal time-step and gravity)
        auto mjcf_option_element = m_MjcfSimulationElement->GetFirstChildOfType( "option" );
        LOCO_CORE_ASSERT( mjcf_option_element, "TMujocoSimulation::_InitializeInternal >>> must have mjcf option element" );
        mjcf_option_element->SetFloat( "timestep", m_FixedTimeStep );
        mjcf_option_element->SetVec3( "gravity", m_Gravity );

        _CollectResourcesFromSingleBodies();
        // _CollectResourcesFromCompounds();
        // _CollectResourcesFromKintrees();
        // _CollectResourcesFromTerrainGenerators();

        // Store the xml-resources for this simulation into disk
        m_MjcfSimulationElement->SaveToXml( "simulation.xml" );

        if ( !TMujocoSimulation::s_HasActivatedMujoco )
        {
            mj_activate( loco::mujoco::LOCO_MUJOCO_LICENSE.c_str() );
            TMujocoSimulation::s_HasActivatedMujoco = true;
        }

        // Load the simulation from the xml-file created above *************************************
        const size_t error_buffer_size = 1000;
        char error_buffer[error_buffer_size];
        m_MjcModel = std::unique_ptr<mjModel, MjcModelDeleter>( mj_loadXML( "simulation.xml", nullptr, error_buffer, error_buffer_size ) );
        if ( !m_MjcModel )
        {
            LOCO_CORE_ERROR( "TMujocoSimulation::_InitializeInternal >>> Couldn't initialize mujoco-API" );
            LOCO_CORE_ERROR( "\tError-message   : {0}", error_buffer );
            return false;
        }
        m_MjcData = std::unique_ptr<mjData, MjcDataDeleter>( mj_makeData( m_MjcModel.get() ) );
        //******************************************************************************************

        for ( auto& single_body_adapter : m_SingleBodyAdapters )
        {
            if ( auto mjc_adapter = dynamic_cast<TMujocoSingleBodyAdapter*>( single_body_adapter.get() ) )
            {
                mjc_adapter->SetMjcModel( m_MjcModel.get() );
                mjc_adapter->SetMjcData( m_MjcData.get() );
            }
        }

        // Take a single step of kinematics computation (to put everything in place)
        mj_kinematics( m_MjcModel.get(), m_MjcData.get() );

        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nq: {0}", m_MjcModel->nq );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nv: {0}", m_MjcModel->nv );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nu: {0}", m_MjcModel->nu );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nbody: {0}", m_MjcModel->nbody );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-njnt: {0}", m_MjcModel->njnt );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-ngeom: {0}", m_MjcModel->ngeom );
        LOCO_CORE_TRACE( "MuJoCo-backend >>> total-nsensor: {0}", m_MjcModel->nsensor );

        return true;
    }

    void TMujocoSimulation::_CollectResourcesFromSingleBodies()
    {
        LOCO_CORE_ASSERT( m_MjcfSimulationElement, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                          there is no mjcf-simulation-element to place the resources in" );

        auto world_body_element = m_MjcfSimulationElement->GetFirstChildOfType( LOCO_MJCF_WORLDBODY_TAG );
        auto assets_element = m_MjcfSimulationElement->GetFirstChildOfType( LOCO_MJCF_ASSET_TAG );

        LOCO_CORE_ASSERT( world_body_element, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                          there is no world-body element in the mjcf-simulation-element" );
        LOCO_CORE_ASSERT( assets_element, "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                          there is no asset element in the mjcf-simulation-element" );

        for ( auto& single_body_adapter : m_SingleBodyAdapters )
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
                        if ( m_MjcfAssetsNames.find( mesh_id ) != m_MjcfAssetsNames.end() )
                            mesh_id_already_cached = true;
                        else
                            m_MjcfAssetsNames.emplace( mesh_id );
                        LOCO_CORE_ASSERT( asset_element->HasAttributeString( "file" ), "TMujocoSimulation::_CollectResourcesFromSingleBodies >>> \
                                          mesh-assets must have a valid mesh-file (but none found on mjcf element" );
                        const std::string mesh_file = asset_element->GetString( "file" );
                        if ( m_MjcfAssetsFilepaths.find( mesh_file ) != m_MjcfAssetsFilepaths.end() )
                            mesh_file_already_cached = true;
                        else
                            m_MjcfAssetsFilepaths.emplace( mesh_file );

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
                        if ( m_MjcfAssetsNames.find( hfield_id ) != m_MjcfAssetsNames.end() )
                            continue; // hfield-id already cached
                        else
                            m_MjcfAssetsNames.emplace( hfield_id );
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

    void TMujocoSimulation::_CollectContacts()
    {
        LOCO_CORE_ASSERT( m_MjcModel, "TMujocoSimulation::_CollectContacts >>> mjModel struct is required \
                          for collecting the contacts from the internal engine" );
        LOCO_CORE_ASSERT( m_MjcModel, "TMujocoSimulation::_CollectContacts >>> mjData struct is required \
                          for collecting the contacts from the internal engine" );

        std::map< std::string, std::vector<TContactData> > detected_contacts;
        ssize_t num_contacts = m_MjcData->ncon;
        for ( ssize_t i = 0; i < num_contacts; i++ )
        {
            auto& mjc_contact_info = m_MjcData->contact[i];
            if ( mjc_contact_info.geom1 == -1 || mjc_contact_info.geom2 == -1 )
            {
                LOCO_CORE_WARN( "TMujocoSimulation::_CollectContacts >>> got a contact without geom-id information" );
                continue;
            }

            const std::string collider_1 = mj_id2name( m_MjcModel.get(), mjOBJ_GEOM, mjc_contact_info.geom1 );
            const std::string collider_2 = mj_id2name( m_MjcModel.get(), mjOBJ_GEOM, mjc_contact_info.geom2 );

            const TVec3 position = { (TScalar) mjc_contact_info.pos[0],
                                     (TScalar) mjc_contact_info.pos[1],
                                     (TScalar) mjc_contact_info.pos[2] };
            const TVec3 normal = { (TScalar) mjc_contact_info.frame[0],
                                   (TScalar) mjc_contact_info.frame[1],
                                   (TScalar) mjc_contact_info.frame[2] };

            if ( detected_contacts.find( collider_1 ) == detected_contacts.end() )
                detected_contacts[collider_1] = std::vector<TContactData>();
            if ( detected_contacts.find( collider_2 ) == detected_contacts.end() )
                detected_contacts[collider_2] = std::vector<TContactData>();

            TContactData contact_1, contact_2;
            contact_1.position = position;  contact_2.position = position;
            contact_1.normal = normal;      contact_2.normal = normal.scaled( -1.0 );
            contact_1.name = collider_2;    contact_2.name = collider_1;

            detected_contacts[collider_1].push_back( contact_1 );
            detected_contacts[collider_2].push_back( contact_2 );
        }

        auto single_bodies = m_ScenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto collider = single_body->collider();
            auto collider_name = collider->name();

            collider->contacts().clear();
            if ( detected_contacts.find( collider_name ) != detected_contacts.end() )
                collider->contacts() = detected_contacts[collider_name];
        }
    }

    void TMujocoSimulation::_PreStepInternal()
    {
        // Make sure recycled objects are not being simulated
        for ( ssize_t i = 0; i < m_SingleBodyAdaptersRecycled.size(); i++ )
            if ( auto mjc_single_body_adapter = dynamic_cast<TMujocoSingleBodyAdapter*>( m_SingleBodyAdaptersRecycled[i].get() ) )
                mjc_single_body_adapter->HideMjcObject();
    }

    void TMujocoSimulation::_SimStepInternal( const TScalar& dt )
    {
        if ( !m_MjcModel )
        {
            LOCO_CORE_WARN( "TMujocoSimulation::_SimStepInternal >>> mjModel structure is required \
                             for taking a simulation step" );
            return;
        }

        if ( !m_MjcData )
        {
            LOCO_CORE_WARN( "TMujocoSimulation::_SimStepInternal >>> mjData structure is required \
                             for taking a simulation step" );
            return;
        }

        const mjtNum sim_step_time = ( dt <= 0 ) ? m_FixedTimeStep : dt;
        const mjtNum sim_start_time = m_MjcData->time;
        while ( ( m_MjcData->time - sim_start_time ) < sim_step_time )
        {
            mj_step( m_MjcModel.get(), m_MjcData.get() );
            m_WorldTime += m_FixedTimeStep;
        }
    }

    void TMujocoSimulation::_PostStepInternal()
    {
        _CollectContacts();
    }

    void TMujocoSimulation::_ResetInternal()
    {
        // Do nothing here, as call to adapters is enough (made in base)
    }

    void TMujocoSimulation::_SetTimeStepInternal( const TScalar& time_step )
    {
        LOCO_CORE_ASSERT( m_MjcModel, "TMujocoSimulation::_SetTimeStepInternal >>> mjModel struct is required \
                          for taking a simulation step, but got nullptr instead" );
        m_MjcModel->opt.timestep = time_step;
    }

    void TMujocoSimulation::_SetGravityInternal( const TVec3& gravity )
    {
        LOCO_CORE_ASSERT( m_MjcModel, "TMujocoSimulation::_SetGravityInternal >>> mjModel struct is required \
                          for taking a simulation step, but got nullptr instead" );
        m_MjcModel->opt.gravity[0] = gravity.x();
        m_MjcModel->opt.gravity[1] = gravity.y();
        m_MjcModel->opt.gravity[2] = gravity.z();
    }

extern "C" TISimulation* simulation_create( loco::TScenario* scenarioRef )
{
    return new loco::mujoco::TMujocoSimulation( scenarioRef );
}

}}