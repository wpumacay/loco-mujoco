
#include <pytysoc_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE( pytysoc, m )
{

    // Common bindings
    PYTYSOC_COMMON_BINDINGS( m )

    m.attr( "__version__" ) = "dev";
}