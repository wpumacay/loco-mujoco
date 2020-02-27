
#include <loco_common_mujoco.h>

namespace loco {
namespace mujoco {

    void MjcModelDeleter::operator()( mjModel* model ) const
    {
        LOCO_CORE_ASSERT( model != nullptr, "MjcModelDeleter >>> should have a valid mjModel reference to release" );
        mj_deleteModel( model );
    }

    void MjcDataDeleter::operator()( mjData* data ) const
    {
        LOCO_CORE_ASSERT( data != nullptr, "MjcDataDeleter >>> should have a valid mjData reference to release" );
        mj_deleteData( data );
    }

}}