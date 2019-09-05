
#include <mujoco_common.h>

namespace tysoc {
namespace mujoco {

    TVec4 quat2MjcfQuat( const TVec4& quat ) 
    { 
        return TVec4( quat.w, quat.x, quat.y, quat.z ); 
    }

}}