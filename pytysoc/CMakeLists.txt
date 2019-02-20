

include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/tysocCore/include" )

pybind11_add_module( pytysoc
                     src/pytysoc_common.cpp
                     src/pytysoc_tests.cpp
                     src/pytysoc_agent_core.cpp
                     src/pytysoc_scenario.cpp
                     src/pytysoc_simulation.cpp
                     src/pytysoc_visualizer.cpp
                     src/pytysoc_runtime.cpp
                     src/pytysoc_bindings.cpp )
target_link_libraries( pytysoc PRIVATE
                       tysocCore )