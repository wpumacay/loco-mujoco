
import numpy as np
import tysoc_bindings as tysoc
import pytysoc


class Door( object ) :

    def __init__( self, name, angle = np.pi / 6.0 ) :
        r"""A simple door object with runtime user-modifiable opening

           |-> rotation axis (hinge|revolute constraint)
           |
           *--------*
           |\------||
           ||\     || -> frame: 4 supporting elements, linked
           || \    ||           together though fixed joints
           ||--\---||
           *--- *---*
            \   |
             \  |  opening-angle
              \ |
               \| -> panel: 1 single element, connected to 
                *           frame through a hinge (revolute-joint)
        """
        self._name = name
        self._angle0 = angle
        self._compound = pytysoc.CreateCompound( 'door_{}'.format( name ) )

        # create the frame of the door (one single part, but separated into fixed linked-components)
        #          top
        #       ---------
        #       |       |
        #  base |       | right
        #       |       |
        #       ---------
        #         bottom
        self._frame_base = self._compound.createRootBody( 'frame_support' )
        self._frame_top, _ = self._frame_support.addBody( 'frame_top' )
        self._frame_bottom, _ = self._frame_support.addBody( 'frame_bottom' )
        self._frame_right, _ self._frame_support.addBody( 'frame_right' )

        # create the panel
        self._panel, self._hinge = self._frame_base.addBody( 'panel', joint_type = pytysoc.Joints.REVOLUTE )

        # set initial configuration of the door
        self.setOpeningAngle( self._angle0 )

    def setOpeningAngle( self, angle ) :
        self._hinge.setAngle( angle )