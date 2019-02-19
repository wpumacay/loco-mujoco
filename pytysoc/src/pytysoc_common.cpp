
#include <pytysoc_common.h>

namespace py = pybind11;

namespace pytysoc
{

    tysoc::TVec2 numpyToVec2( py::array_t<TScalar>& nparray )
    {
        auto _bufferInfo = nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;
        if ( _bufferInfo.size != 2 )
        {
            std::cout << "WARNING> vec2 conversion requires an array with 2 elements" << std::endl;
            return tysoc::TVec2();
        }

        return tysoc::TVec2( _data[0], _data[1] );
    }

    tysoc::TVec3 numpyToVec3( py::array_t<TScalar>& nparray )
    {
        auto _bufferInfo = nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;
        if ( _bufferInfo.size != 3 )
        {
            std::cout << "WARNING> vec3 conversion requires an array with 3 elements" << std::endl;
            return tysoc::TVec3();
        }

        return tysoc::TVec3( _data[0], _data[1], _data[2] );
    }

    tysoc::TVec4 numpyToVec4( py::array_t<TScalar>& nparray )
    {
        auto _bufferInfo = nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;
        if ( _bufferInfo.size != 4 )
        {
            std::cout << "WARNING> vec4 conversion requires an array with 3 elements" << std::endl;
            return tysoc::TVec4();
        }

        return tysoc::TVec4( _data[0], _data[1], _data[2], _data[3] );
    }

    tysoc::TMat3 numpyToMat3( py::array_t<TScalar>& nparray )
    {
        auto _bufferInfo = nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;
        if ( _bufferInfo.size != 9 )
        {
            std::cout << "WARNING> mat3 conversion requires an array with 9 elements" << std::endl;
            return tysoc::TMat3();
        }
        // std::cout << "LOG> size: " << _bufferInfo.size << std::endl;
        // std::cout << "LOG> ndim: " << _bufferInfo.ndim << std::endl;
        // for ( size_t q = 0; q < _bufferInfo.ndim; q++ )
        // {
        //     std::cout << "LOG> shape[" << q << "]: " << _bufferInfo.shape[q] << std::endl;
        // }

        return tysoc::TMat3( _data[0], _data[1], _data[2],
                             _data[3], _data[4], _data[5],
                             _data[6], _data[7], _data[8] );
    }

    tysoc::TMat4 numpyToMat4( py::array_t<TScalar>& nparray )
    {
        auto _bufferInfo = nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;
        if ( _bufferInfo.size != 16 )
        {
            std::cout << "WARNING> mat4 conversion requires an array with 16 elements" << std::endl;
            return tysoc::TMat4();
        }
        // std::cout << "LOG> size: " << _bufferInfo.size << std::endl;
        // std::cout << "LOG> ndim: " << _bufferInfo.ndim << std::endl;
        // for ( size_t q = 0; q < _bufferInfo.ndim; q++ )
        // {
        //     std::cout << "LOG> shape[" << q << "]: " << _bufferInfo.shape[q] << std::endl;
        // }

        return tysoc::TMat4( _data[0], _data[1], _data[2], _data[3],
                             _data[4], _data[5], _data[6], _data[7],
                             _data[8], _data[9], _data[10], _data[11],
                             _data[12], _data[13], _data[14], _data[15] );
    }


    py::array_t<TScalar> vec2ToNumpy( const tysoc::TVec2& vec2 )
    {
        auto _nparray = py::array_t<TScalar>( 2 );
        auto _bufferInfo = _nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;

        _data[0] = vec2.x;
        _data[1] = vec2.y;

        return _nparray;
    }

    py::array_t<TScalar> vec3ToNumpy( const tysoc::TVec3& vec3 )
    {
        auto _nparray = py::array_t<TScalar>( 3 );
        auto _bufferInfo = _nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;

        _data[0] = vec3.x;
        _data[1] = vec3.y;
        _data[2] = vec3.z;

        return _nparray;
    }

    py::array_t<TScalar> vec4ToNumpy( const tysoc::TVec4& vec4 )
    {
        auto _nparray = py::array_t<TScalar>( 4 );
        auto _bufferInfo = _nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;

        _data[0] = vec4.x;
        _data[1] = vec4.y;
        _data[2] = vec4.z;
        _data[3] = vec4.w;

        return _nparray;
    }

    // from: https://github.com/pybind/pybind11/issues/27
    py::array_t<TScalar> mat3ToNumpy( const tysoc::TMat3& mat3 )
    {
        std::vector< ssize_t > _shape = { 3, 3 };
        std::vector< ssize_t > _strides = { sizeof( TScalar ), sizeof( TScalar ) };

        return py::array( py::buffer_info( (TScalar*) mat3.buff,
                                           sizeof( TScalar ),
                                           py::format_descriptor<TScalar>::value,
                                           2, _shape, _strides ) );
    }

    py::array_t<TScalar> mat4ToNumpy( const tysoc::TMat4& mat4 )
    {
        std::vector< ssize_t > _shape = { 4, 4 };
        std::vector< ssize_t > _strides = { sizeof( TScalar ), sizeof( TScalar ) };

        return py::array( py::buffer_info( (TScalar*) mat4.buff,
                                           sizeof( TScalar ),
                                           py::format_descriptor<TScalar>::value,
                                           2, _shape, _strides ) );
    }

    void test_numpyToVec2( py::array_t<TScalar>& nparray )
    {
        auto _vec2 = numpyToVec2( nparray );
        std::cout << "LOG> vec2: " << tysoc::TVec2::toString( _vec2 ) << std::endl;
    }

    void test_numpyToVec3( py::array_t<TScalar>& nparray )
    {
        auto _vec3 = numpyToVec3( nparray );
        std::cout << "LOG> vec3: " << tysoc::TVec3::toString( _vec3 ) << std::endl;
    }

    void test_numpyToVec4( py::array_t<TScalar>& nparray )
    {
        auto _vec4 = numpyToVec4( nparray );
        std::cout << "LOG> vec4: " << tysoc::TVec4::toString( _vec4 ) << std::endl;
    }

    void test_numpyToMat3( py::array_t<TScalar>& nparray )
    {
        auto _mat3 = numpyToMat3( nparray );
        std::cout << "LOG> mat3: " << tysoc::TMat3::toString( _mat3 ) << std::endl;
    }

    void test_numpyToMat4( py::array_t<TScalar>& nparray )
    {
        auto _mat4 = numpyToMat4( nparray );
        std::cout << "LOG> mat4: " << tysoc::TMat4::toString( _mat4 ) << std::endl;
    }

    std::vector<TScalar> numpyToVecArray( py::array_t<TScalar>& nparray )
    {
        auto _bufferInfo = nparray.request();
        auto _data = ( TScalar* ) _bufferInfo.ptr;

        std::vector<TScalar> _vecarray;
        for ( size_t q = 0; q < _bufferInfo.size; q++ )
        {
            _vecarray.push_back( _data[q] );
        }

        return _vecarray;
    }

    void test_numpyToVecArray( py::array_t<TScalar>& nparray )
    {
        auto _vecarray = numpyToVecArray( nparray );
        std::cout << "LOG> std::vector [ ";
        for ( size_t q = 0; q < _vecarray.size(); q++ )
        {
            std::cout << _vecarray[q] << " ";
        }
        std::cout << "]" << std::endl;
    }

}