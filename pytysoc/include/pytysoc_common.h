
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <tysoc_common.h>

namespace py = pybind11;

namespace pytysoc
{


    tysoc::TVec2 numpyToVec2( py::array_t<TScalar>& array );
    tysoc::TVec3 numpyToVec3( py::array_t<TScalar>& array );
    tysoc::TVec4 numpyToVec4( py::array_t<TScalar>& array );

    tysoc::TMat3 numpyToMat3( py::array_t<TScalar>& array );
    tysoc::TMat4 numpyToMat4( py::array_t<TScalar>& array );

    py::array_t<TScalar> vec2ToNumpy( const tysoc::TVec2& vec2 );
    py::array_t<TScalar> vec3ToNumpy( const tysoc::TVec3& vec3 );
    py::array_t<TScalar> vec4ToNumpy( const tysoc::TVec4& vec4 );

    py::array_t<TScalar> mat3ToNumpy( const tysoc::TMat3& mat3 );
    py::array_t<TScalar> mat4ToNumpy( const tysoc::TMat4& mat4 );


    void test_numpyToVec2( py::array_t<TScalar>& array );
    void test_numpyToVec3( py::array_t<TScalar>& array );
    void test_numpyToVec4( py::array_t<TScalar>& array );

    void test_numpyToMat3( py::array_t<TScalar>& array );
    void test_numpyToMat4( py::array_t<TScalar>& array );

}

#define PYTYSOC_COMMON_BINDINGS(m) \
    m.def( "test_numpyToVec2", &pytysoc::test_numpyToVec2 );\
    m.def( "test_numpyToVec3", &pytysoc::test_numpyToVec3 );\
    m.def( "test_numpyToVec4", &pytysoc::test_numpyToVec4 );\
    m.def( "test_numpyToMat3", &pytysoc::test_numpyToMat3 );\
    m.def( "test_numpyToMat4", &pytysoc::test_numpyToMat4 );
