
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

// @TODO|@CHECK|@WIP : for now we are just trying without dynamic loading
#include <tysocMjc.h>
// #include <tysocBullet.h> ????

namespace pytysoc
{

    struct TRuntimeFunctions
    {
        
    };

    void loadRuntime( const std::string& type );


    class TRuntimeWrapper
    {

        private :



    };



}