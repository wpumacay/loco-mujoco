
import numpy as np
import pytysoc

if __name__ == '__main__' :

    box = pytysoc.createSingleBody( "box_0", pytysoc.Shape.BOX, size = [ 0.2, 0.2, 0.2 ] )
    sphere = pytysoc.createSingleBody( "sphere_0", pytysoc.Shape.SPHERE, radius = 0.1 )
    monkey = pytysoc.createSingleBody( "mesh_0", pytysoc.Shape.MESH, path = pytysoc.Resources.Meshes.MONKEY, scale = [ 0.1, 0.1, 0.1 ] )