
#include <chrono>
#include <runtime.h>
#include <mujoco_config.h>
#include <random>

tysoc::TCompound* createDoorVersion1( const std::string& name, const tysoc::TVec3& position )
{
    tysoc::TCompound* _compound = new tysoc::TCompound( name, 
                                                        position, 
                                                        tysoc::TMat3::fromEuler( { 0.0f, 0.0f, 0.0f } ),
                                                        tysoc::eDynamicsType::DYNAMIC );

    /* create compound bodies separately, compose them and add to the compound afterwards *********/

    auto _frame_0_coldata = tysoc::TCollisionData();
    _frame_0_coldata.type = tysoc::eShapeType::BOX;
    _frame_0_coldata.size = { 0.2f, 0.2f, 2.0f };
    auto _frame_0_visdata = tysoc::TVisualData();
    _frame_0_visdata.type = tysoc::eShapeType::BOX;
    _frame_0_visdata.size = { 0.2f, 0.2f, 2.0f };
    _frame_0_visdata.ambient = { 0.4f, 0.3f, 0.2f };
    _frame_0_visdata.diffuse = { 0.4f, 0.3f, 0.2f };
    _frame_0_visdata.specular = { 0.4f, 0.3f, 0.2f };
    _frame_0_visdata.shininess = 50.0f;
    auto _frame_0_body_data = tysoc::TBodyData();
    _frame_0_body_data.collision = _frame_0_coldata;
    _frame_0_body_data.visual = _frame_0_visdata;
    auto _frame_0_local_tf = tysoc::TMat4::fromPositionAndRotation( { 0.0f, -0.5f, 1.0f }, tysoc::TMat3() );
    auto _frame_0_body = _compound->createRootBody( "frame_0", 
                                                    _frame_0_body_data,
                                                    _frame_0_local_tf );

    auto _frame_1_coldata = tysoc::TCollisionData();
    _frame_1_coldata.type = tysoc::eShapeType::BOX;
    _frame_1_coldata.size = { 0.2f, 0.8f, 0.2f };
    auto _frame_1_visdata = tysoc::TVisualData();
    _frame_1_visdata.type = tysoc::eShapeType::BOX;
    _frame_1_visdata.size = { 0.2f, 0.8f, 0.2f };
    _frame_1_visdata.ambient = { 0.4f, 0.3f, 0.2f };
    _frame_1_visdata.diffuse = { 0.4f, 0.3f, 0.2f };
    _frame_1_visdata.specular = { 0.4f, 0.3f, 0.2f };
    _frame_1_visdata.shininess = 50.0f;
    auto _frame_1_body_data = tysoc::TBodyData();
    _frame_1_body_data.collision = _frame_1_coldata;
    _frame_1_body_data.visual = _frame_1_visdata;
    auto _frame_1_joint_data = tysoc::TJointData();
    _frame_1_joint_data.type = tysoc::eJointType::FIXED;
    _frame_1_joint_data.localTransform = tysoc::TMat4(); // identity (same ref-frame as body's frame)
    auto _frame_1_local_tf = tysoc::TMat4::fromPositionAndRotation( { 0.0f, 0.5f, 0.9f }, tysoc::TMat3() );
    auto _frame_1_body = new tysoc::TCompoundBody( "frame_1",
                                                   _frame_1_body_data,
                                                   _frame_1_joint_data,
                                                   _frame_0_body,
                                                   _frame_1_local_tf.getPosition(),
                                                   _frame_1_local_tf.getRotation() );

    auto _frame_2_coldata = tysoc::TCollisionData();
    _frame_2_coldata.type = tysoc::eShapeType::BOX;
    _frame_2_coldata.size = { 0.2f, 0.2f, 2.0f };
    auto _frame_2_visdata = tysoc::TVisualData();
    _frame_2_visdata.type = tysoc::eShapeType::BOX;
    _frame_2_visdata.size = { 0.2f, 0.2f, 2.0f };
    _frame_2_visdata.ambient = { 0.4f, 0.3f, 0.2f };
    _frame_2_visdata.diffuse = { 0.4f, 0.3f, 0.2f };
    _frame_2_visdata.specular = { 0.4f, 0.3f, 0.2f };
    _frame_2_visdata.shininess = 50.0f;
    auto _frame_2_body_data = tysoc::TBodyData();
    _frame_2_body_data.collision = _frame_2_coldata;
    _frame_2_body_data.visual = _frame_2_visdata;
    auto _frame_2_joint_data = tysoc::TJointData();
    _frame_2_joint_data.type = tysoc::eJointType::FIXED;
    _frame_2_joint_data.localTransform = tysoc::TMat4(); // identity (same ref-frame as body's frame)
    auto _frame_2_local_tf = tysoc::TMat4::fromPositionAndRotation( { 0.0f, 0.5f, -0.9f }, tysoc::TMat3() );
    auto _frame_2_body = new tysoc::TCompoundBody( "frame_2",
                                                   _frame_2_body_data,
                                                   _frame_2_joint_data,
                                                   _frame_1_body,
                                                   _frame_2_local_tf.getPosition(),
                                                   _frame_2_local_tf.getRotation() );

    auto _panel_coldata = tysoc::TCollisionData();
    _panel_coldata.type = tysoc::eShapeType::BOX;
    _panel_coldata.size = { 1.2f, 0.1f, 2.0f };
    auto _panel_visdata = tysoc::TVisualData();
    _panel_visdata.type = tysoc::eShapeType::BOX;
    _panel_visdata.size = { 1.2f, 0.1f, 2.0f };
    _panel_visdata.ambient = { 0.4f, 0.3f, 0.5f };
    _panel_visdata.diffuse = { 0.4f, 0.3f, 0.5f };
    _panel_visdata.specular = { 0.4f, 0.3f, 0.5f };
    _panel_visdata.shininess = 50.0f;
    auto _panel_body_data = tysoc::TBodyData();
    _panel_body_data.collision = _panel_coldata;
    _panel_body_data.visual = _panel_visdata;
    auto _panel_joint_data = tysoc::TJointData();
    _panel_joint_data.type = tysoc::eJointType::REVOLUTE;
    _panel_joint_data.axis = { 0.0f, 0.0f, 1.0f };
    _panel_joint_data.limits = { -0.5f * TYSOC_PI, 0.5F * TYSOC_PI };
    _panel_joint_data.localTransform = tysoc::TMat4::fromPositionAndRotation( { 0.1f, -0.1f, 0.0f }, tysoc::TMat3() );
    auto _panel_local_tf = tysoc::TMat4::fromPositionAndRotation( { 0.7f, -0.15f, 0.0f }, tysoc::TMat3() );
    auto _panel_body = new tysoc::TCompoundBody( "panel",
                                                 _panel_body_data,
                                                 _panel_joint_data,
                                                 _frame_0_body,
                                                 _panel_local_tf.getPosition(),
                                                 _panel_local_tf.getRotation() );

    // add all bodies (the compound is just a handy container)
    _compound->addCompoundBody( std::unique_ptr< tysoc::TCompoundBody >( _frame_1_body ) );
    _compound->addCompoundBody( std::unique_ptr< tysoc::TCompoundBody >( _frame_2_body ) );
    _compound->addCompoundBody( std::unique_ptr< tysoc::TCompoundBody >( _panel_body ) );

    /**********************************************************************************************/

    return _compound;
}

//// tysoc::TCompound* createDoorVersion2( const std::string& name, const tysoc::TVec3& position )
//// {
////     tysoc::TCompound* _compound = new tysoc::TCompound( name, position, tysoc::TMat3::fromEuler( { 0.0f, 0.0f, 0.0f } ) );
//// 
////     /* create compound bodies recursively, using parents to create their children *****************/
//// 
////     auto _frame_0_body = _compound->createRootBody( "frame_0",
////                                                     tysoc::eShapeType::BOX,
////                                                     { 0.2f, 0.2f, 2.0f },
////                                                     { 0.4f, 0.3f, 0.2f },
////                                                     tysoc::TMat4( { 0.0f, -0.5f, 1.0f }, tysoc::TMat3() ) );
//// 
////     auto _frame_1_body_joint_pair = _frame_0_body->addBodyJointPair( "frame_1",
////                                                                      tysoc::eShapeType::BOX,
////                                                                      { 0.2f, 0.8f, 0.2f },
////                                                                      tysoc::TMat4( { 0.0f, 0.5f, 0.9f }, tysoc::TMat3() ),
////                                                                      tysoc::eJointType::FIXED,
////                                                                      { 0.0f, 0.0f, 0.0f }, // axis not required for fixed-joints
////                                                                      { 0.0f, 0.0f }, // limits not required for fixed-joints
////                                                                      tysoc::TMat4() ); // location in same frame as owner body
////     auto _frame_1_body = _frame_1_body_joint_pair.first;
//// 
////     auto _frame_2_body_joint_pair = _frame_0_body->addBodyJointPair( "frame_2",
////                                                                      tysoc::eShapeType::BOX,
////                                                                      { 0.2f, 0.2f, 2.0f },
////                                                                      tysoc::TMat4( { 0.0f, 0.5f, -0.9f }, tysoc::TMat3() ),
////                                                                      tysoc::eJointType::FIXED,
////                                                                      { 0.0f, 0.0f, 0.0f }, // axis not required for fixed-joints
////                                                                      { 0.0f, 0.0f }, // limits not required for fixed-joints
////                                                                      tysoc::TMat4() ); // location in same frame as owner body
////     auto _frame_2_body = _frame_2_body_joint_pair.first;
//// 
////     // add all bodies (the compound is just a handy container)
////     _compound->addCompoundBody( std::unique_ptr< tysoc::TCompoundBody >( _frame_1_body ) );
////     _compound->addCompoundBody( std::unique_ptr< tysoc::TCompoundBody >( _frame_2_body ) );
//// 
////     /**********************************************************************************************/
//// 
////     /**********************************************************************************************/
//// 
////     return _compound;
//// }

int main()
{
    auto _terrainGenStatic = new tysoc::TStaticTerrainGenerator( "terrainGen0" );
    _terrainGenStatic->createPrimitive( "plane", 
                                        { 10.0f, 10.0f, 0.2f }, 
                                        { 0.0f, 0.0f, -1.0f },
                                        tysoc::TMat3(),
                                        { 0.2f, 0.3f, 0.4f },
                                        "built_in_chessboard" );

    auto _scenario = new tysoc::TScenario();
    _scenario->addTerrainGenerator( _terrainGenStatic );

    auto _compoundDoor = createDoorVersion1( "door", { 0.0f, 0.0f, 2.0f } );
    _scenario->addCompound( _compoundDoor );

    auto _runtime = new tysoc::TRuntime( tysoc::config::physics::MUJOCO, 
                                         tysoc::config::rendering::GLVIZ );

    auto _simulation = _runtime->createSimulation( _scenario );
    _simulation->initialize();

    auto _visualizer = _runtime->createVisualizer( _scenario );
    _visualizer->initialize();

    while ( _visualizer->isActive() )
    {
        auto _start = std::chrono::high_resolution_clock::now();

        if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_P ) )
            _simulation->togglePause();
        else if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_R ) )
            _simulation->reset();
        else if ( _visualizer->checkSingleKeyPress( tysoc::keys::KEY_ESCAPE ) )
            break;

        _simulation->step();

        _visualizer->update();

        auto _duration = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::high_resolution_clock::now() - _start );
        //// std::cout << "step-time: " << _duration.count() << " ||| fps: " << ( 1000.0 / _duration.count() ) << std::endl;
    }

    _runtime->destroyVisualizer();
    _runtime->destroySimulation();
    _visualizer = nullptr;

    return 0;
}
