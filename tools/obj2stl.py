
# converts .obj files in current working
# directory into .stl using assimp
# to use, install assimp:
# sudo apt-get install assimp-utils
# files should not include '.' in it (appart
# from the file extension)

import os
import sys
import subprocess as sp

try :
    sp.call( ['assimp', 'version'] )
except :
    print( 'There was an error calling assimp. Is it installed?' )
    print( 'Run: sudo apt-get install assimp-utils' )
    sys.exit( -1 )


_cwd = os.getcwd()
print( 'working in folder: ', _cwd )

# grab files in cwd
_files      = os.listdir()
_filesSet   = set( _files )

# convert .obj files
for _file in _files :

    if '.obj' not in _file :
        # skip if not .obj file
        continue

    _filename = _file.split( '.' )[0]
    if ( _filename + '.stl' ) in _filesSet :
        # skip already existing .stl file
        continue

    print( 'converting: ', _file )
    sp.call( [ 'assimp', 
               'export', 
               _filename + '.obj',
               _filename + '.stl' ] )