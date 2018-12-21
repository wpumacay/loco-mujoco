
#include <tysocCustomVizCommon.h>

namespace tysoc{
namespace viz{

    engine::LMat4 fromTMat3( const TMat3& mat )
    {
        engine::LMat4 _res;

        _res.buff[0] = mat.buff[0];
        _res.buff[1] = mat.buff[1];
        _res.buff[2] = mat.buff[2];

        _res.buff[4] = mat.buff[3];
        _res.buff[5] = mat.buff[4];
        _res.buff[6] = mat.buff[5];

        _res.buff[8]  = mat.buff[6];
        _res.buff[9]  = mat.buff[7];
        _res.buff[10] = mat.buff[8];

        return _res;
    }

    engine::LVec3 fromTVec3( const TVec3& vec )
    {
        engine::LVec3 _res;

        _res.x = vec.x;
        _res.y = vec.y;
        _res.z = vec.z;

        return _res;
    }


}}