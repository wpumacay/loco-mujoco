
#include <pytysoc_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE( pytysoc, m )
{
    // Some code to run when importing the module
    tysoc::TModelLoader::GetInstance();

    // Common bindings
    PYTYSOC_COMMON_BINDINGS( m )
    // Tests bindings
    PYTYSOC_TESTS_BINDINGS( m )
    // Core agent bindings
    PYTYSOC_CORE_AGENT_BINDINGS( m )
    // Scenario bindings
    PYTYSOC_SCENARIO_BINDINGS( m )
    // visualizer bindings
    PYTYSOC_VISUALIZER_BINDINGS( m )
    // simulation bindings
    PYTYSOC_SIMULATION_BINDINGS( m )
    // runtime bindings
    PYTYSOC_RUNTIME_BINDINGS( m )

    m.attr( "__version__" ) = "dev";
}