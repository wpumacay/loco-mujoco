
# Some dev notes I made to implement/refactor/design some features

## About terrain "primitives" and changes in the design for january

Well, this is from the bullet concretion, but should apply here as well :

```c++
    // @CHECK: This part should be generalized in order to accomodate ...
    // other types of primitives as needed (like meshes, add joints for doors, etc.)
    // For example, to make a door, instead of having a geomtype, we could use a generic type ...
    // like bulletPrimitiveType, which would be a "meta" for object creation. We then dispatch this ...
    // request of creation to an appropiate factory that builds the door for us, and we just wrapped it
    // For a mesh, it could be the same. The underlying data structure we want to wrap already has the info ...
    // (must have it) to build the mesh, so we only need to request that to the factory and voila, we have ...
    // our mesh. The same should apply for other types of terrain objects : the factories are the providers ...
    // for object creation, and they only need the information of what we want to build (and they could be ...
    // abstract factories, and could be instantiated for a concrete API implementation, either bullet or mujoco)
```

I want to support more features and tools to make the user happy :D. In my case, I wanted to reproduce the 
DARPA scenarios and tasks, so there are lots of extra features that are needed that are not simple primitives.

In order to accomodate this I need to stop using primitives only and be able to create more complex 
"components" for the terrain. Sure, some might be primitives (like boxes or spheres that could be around for some tasks)
but ultimately this should not be the only ones.

This brings a nice way (so far) to decouple away the different types of terrain. The abstract features should request
creation from a specific factory, and these then assembled, stored and wrapped in the appropiate wrappers. So far, I think
by using this we could still use the same terraingenwrapper for all "components", as long as all of them only require minimal
functionality from the concrete implementation. This is what I mean :

* Suppose that I want to create doors, some obstacles like rocks, some stairs and add a jigglypuff here and there.
* Oh, and also, some long path that is procedurally generated, which would take you some other place.
* The underlying position and orientation should be handled by the abstract underlying functionality :
    * In the case of the doors, they don't move, and they turn as the agent interacts, but the user does not move it
    * Stairs remain in place, and some other terrain-static features hardly move (like a heightfield or similar)
    * Rocks only move as you interact with them, and their position is updated from the engine back to the underlying abstract object.
    * Jigglypuffs should not move if are part of the terrain. If not, then they go somewhere else (like agent/npc wrappers).
    * Procedurally generated terrain moves as the underlying functionality wants them to move, and their position is changed in
      the engine by the wrapper.

So, TERRAIN in the sense we are dealing with, is mainly "static" terrain (in the sense that the user does not control them) and
also "procedural" terrain (in the sense that the underlying logic moves things around). This means we are only dealing with two types
of wrapper here, one for static-terrain generators, and one for procedural-terrain generators.

## About object pool

I think that I should make a separate object pool class that could have the information necessary when trying to recycle and reuse
an object from the terrain. I came up with this because I need to make sure that objects in the pool satisfy some certain properties,
and a separate queue with its logic being handled by a non-friendly object don't make it that clean (I guess). Some properties include
making the objects be in a certain position, deactivating them (if possible), resizing them, etc.