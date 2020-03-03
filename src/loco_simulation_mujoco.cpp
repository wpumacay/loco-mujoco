
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

        _CreateSingleBodyAdapters();
        //// _CreateCompoundAdapters();
        //// _CreateKintreeAdapters();
        //// _CreateTerrainGeneratorAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TMujocoSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TMujocoSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TMujocoSimulation::_CreateSingleBodyAdapters()
    {
        auto single_bodies = m_scenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TMujocoSingleBodyAdapter>( single_body );
            single_body->SetAdapter( single_body_adapter.get() );
            m_singleBodyAdapters.push_back( std::move( single_body_adapter ) );

            auto collider = single_body->collision();
            LOCO_CORE_ASSERT( collider, "TMujocoSimulation::_CreateSingleBodyAdapters >>> single-body {0} \
                              must have an associated collider", single_body->name() );

            auto collider_adapter = std::make_unique<TMujocoCollisionAdapter>( collider );
            collider->SetAdapter( collider_adapter.get() );
            m_collisionAdapters.push_back( std::move( collider_adapter ) );
        }
    }

    TMujocoSimulation::~TMujocoSimulation()
    {
        m_mjcModel = nullptr;
        m_mjcData = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TMujocoSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TMujocoSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TMujocoSimulation::_InitializeInternal()
    {
        // Create empty mjcf-xml structure to store the simulation resources
        const std::string empty_mjcf_str =
        R"( <mujoco>
                <compiler inertiafromgeom="true" coordinate="local" angle="degree"/>
                <size njmax="10000" nconmax="40000"/>
                <statistic extent="2" center="0 0 1"/>
                <asset>
                  <!-- place assets here -->
                </asset>
            </mujoco> )";
        auto simulation_element = parsing::TElement::CreateFromXmlString( parsing::eSchemaType::MJCF, empty_mjcf_str );

        auto world_body = simulation_element->Add( LOCO_MJCF_WORLDBODY_TAG );
        for ( auto& single_body_adapter : m_singleBodyAdapters )
            if ( auto mjc_adapter = dynamic_cast<TMujocoSingleBodyAdapter*>( single_body_adapter.get() ) )
                if ( auto mjcf_element = mjc_adapter->element_resources() )
                    world_body->Add( parsing::TElement::CloneElement( mjcf_element ) );

        // Store the xml-resources for this simulation into disk
        simulation_element->SaveToXml( "simulation.xml" );

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

    void TMujocoSimulation::_PreStepInternal()
    {

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

}}