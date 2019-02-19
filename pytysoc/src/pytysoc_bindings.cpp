
#include <pytysoc_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE( pytysoc, m )
{
    // Some code to run when importing the module
    tysoc::TModelLoader::GetInstance();

    // Common bindings
    PYTYSOC_COMMON_BINDINGS( m )
    // Core agent bindings
    PYTYSOC_CORE_AGENT_BINDINGS( m )

    m.attr( "__version__" ) = "dev";
}