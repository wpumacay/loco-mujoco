
# computes an approximate of the inertia ...
# of a given mesh, given the mass of the ...
# object. Approximates to a box or sphere ...
# according to the user choice

# depends on pymesh:
# Clone from repository:
#   git clone https://github.com/PyMesh/PyMesh.git
# Grab submodules:
#   cd PyMesh
#   git submodule update --init
# Build using setup.py (use an anaconda env)
#   source activate tysoc
#   setup.py build
#   setup.py install

# I'm just using the library to compute bounding boxes ...
# as I could not find an easier solution, and vtk is a bit ...
# funky with the installation.
# Ideally I would like to compute the inertia from the mesh ...
# itself using something like approximating with tetrahedra, ...
# or similar. I will look up this later, as for now I want ...
# just a rough approximation to make things work properly.

import os
import sys
import json

try :
    import pymesh
except :
    print( 'There was an error importing pymesh. Is it installed?' )
    print( 'Check installation instructions in this file header or ...' )
    print( 'Go to https://github.com/PyMesh/PyMesh for more information' )
    sys.exit( -1 )


DEFAULT_DENSITY = 1000.0 # density of water in SI units (1000 kg/m^3)

def _computeInertiaApproxBox( mesh, mass ) :
    _meshBoundingBox = mesh.bbox
    # approximate the geometry as if it were a box ...
    # that encloses the mesh, with the same mass as the ...
    # actual object.
    _dx = _meshBoundingBox[1][0] - _meshBoundingBox[0][0]
    _dy = _meshBoundingBox[1][1] - _meshBoundingBox[0][1]
    _dz = _meshBoundingBox[1][2] - _meshBoundingBox[0][2]
    _ixx = ( 1. / 12. ) * mass * ( _dy ** 2 + _dz ** 2 )
    _iyy = ( 1. / 12. ) * mass * ( _dx ** 2 + _dz ** 2 )
    _izz = ( 1. / 12. ) * mass * ( _dx ** 2 + _dy ** 2 )
    _ixy = 0.0
    _ixz = 0.0
    _iyz = 0.0

    return [ _ixx, _iyy, _izz, _ixy, _ixz, _iyz ]


"""
    Compute inertia with default density (no mass given)
"""
def computeInertiaNoMass( meshStlName ) :
    _mesh = pymesh.load_mesh( meshStlName + '.stl' )
    _meshVolume = _mesh.volume
    _meshMass   = _meshVolume * DEFAULT_DENSITY

    return _computeInertiaApproxBox( _mesh, _meshMass ), _meshMass

"""
    Compute inertia with mass given
"""
def computeInertiaWithMass( meshStlName, mass ) :
    _mesh = pymesh.load_mesh( meshStlName + '.stl' )
    
    return _computeInertiaApproxBox( _mesh, mass )

"""
    Compute approx. inertia for every
    model in the current directory (from stl)
"""
def main():
    _filesStl = []
    _files = os.listdir()
    
    for _file in _files :
        if '.stl' in _file :
            _filename = _file.split( '.stl' )[0]
            _filesStl.append( _filename )

    # grab list of meshes with given masses
    _meshesWithMass = {}
    if 'mass.json' in _files :
        with open( 'mass.json' ) as _fhandle :
            _meshesWithMass = json.load( _fhandle )

    _results = {}
    for _meshStl in _filesStl :
        _inertia, _mass = computeInertiaNoMass( _meshStl )
        _res = { 'default' :  { 'mass' : _mass,
                                'inertia' : _inertia } }
        _results[ _meshStl ] = _res

        if _meshStl in _meshesWithMass :
            _mass = _meshesWithMass[ _meshStl ]
            _inertia = computeInertiaWithMass( _meshStl, _mass )
            _results[ _meshStl ]['withMass'] = { 'mass' : _mass,
                                                 'inertia' : _inertia }

    ## print( 'results: ' )
    ## print( _results )

    with open( 'inertia.json', 'w' ) as _fhandle :
        json.dump( _results, _fhandle, indent = 4 )

if __name__ == '__main__' :
    main()
