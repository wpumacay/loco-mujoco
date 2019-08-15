
# Project Updates

### 12, nov, 2018

![terrain-progress-1](https://media.giphy.com/media/psnfTo2l32SxR9lb5h/giphy.gif)

* Implemented base functionality to interact with mujoco objects. (ext/mjcint)

    1. Change body position
    2. Set actuator ctrls

* Implemented mjcf object handler (ext/mjcf)

    1. Allows to parse mjcf into basic object model
    2. Allows to create models using this basic object model

### 13, nov, 2018

![terrain-progress-2](https://media.giphy.com/media/uiKS25QJOJnjWpoEHo/giphy.gif)

* Implemented base abstract terrain generation. (ext/tysocCore)

    1. implemented basic structure of the abstract terrain generation library
    2. implemented sections-type terraing generation

* Refactored repo and updated design

![design-v0](../_imgs/img_tysoc_design_v0.jpg)

### 21, nov, 2018

![terrain-progress-3](https://media.giphy.com/media/XHUj9S0F4GgVAmZW0p/giphy.gif)

![terrain-progress-4](https://media.giphy.com/media/3dgkXuLXHRIpdMTCZy/giphy.gif)

![terrain-progress-5](https://media.giphy.com/media/7vAgc1vznGDVvlpFMm/giphy.gif)

![terrain-progress-6](https://media.giphy.com/media/fwZ0REGoOVj57aJdWE/giphy.gif)

* Implemented framework :

    1. Implemented the full base framework and added wrappers for mujoco
    2. Added perlin noise generator
    3. Added walker and humanoid template

### 23, nov, 2018

* Integrated custom rendering engine into a decoupled visualizer : the visualizer is working and is decoupled of the concrete physics engine used. It updates its data from the underlying data being used by the abstract API, whose contents are written by the concrete implementations.

![terrain-progress-7](https://media.giphy.com/media/nnKODYDjpB8Vsj49sA/giphy.gif)

![terrain-progress-8](https://media.giphy.com/media/5h29BWEJxfI67e5ghS/giphy.gif)

### 24, nov, 2018

* Added sensors (agent intrinsic and terrain extrinsic) to the library

![terrain-progress-9](https://media.giphy.com/media/fMzywtksuU42jHWvNi/giphy.gif)

![terrain-progress-10](https://media.giphy.com/media/3gPDeNpim4gMvOnqQe/giphy.gif)

### 25, nov, 2018

* Added terrain generators to replicate the environments from [here](https://www.youtube.com/watch?v=hx_bgoTF7bs)

![terrain-progress-11](https://media.giphy.com/media/loG8Nf202bsfw8dJPO/giphy.gif)

![terrain-progress-12](https://media.giphy.com/media/Zvn0VIdbm52zf2SXkz/giphy.gif)

![terrain-progress-13](https://media.giphy.com/media/Bp5mbsdlZFbwsMIdaK/giphy.gif)

![terrain-progress-14](https://media.giphy.com/media/3rg0hDmYXKNnUZmUej/giphy.gif)

![terrain-progress-15](https://media.giphy.com/media/euGmWrsjhLgDPe6GmW/giphy.gif)

### 13, dic, 2018

* Made various changes after poster presentation :

    1. Implemented Kinematic tree agents
    2. Modified agent API
    3. Added small UI
    4. Fixed sensors for new agent API
    5. Refactored a bit

![tysoc-mjc-progress-1](https://media.giphy.com/media/ZcaynbjABz69Zyj9n9/giphy.gif)

![tysoc-mjc-progress-2](https://media.giphy.com/media/loMkXQHDRSeHC8s2dy/giphy.gif)

![tysoc-mjc-progress-3](https://media.giphy.com/media/9S1zriY4MMt8LjOoSq/giphy.gif)

### dic, 2018

* Added starting support for urdf
* Refactored code: parsers
* Added joint sensors
* Added support for mujoco visualizer
* ... (wil check commits later)

### jan, 2019

* Added support for urdf (still some fixes needed)
* Refactored code: design of the core agent functionality
* ... (wil check commits later)

### feb, 2019

* Added support for the rlsim format (from [here](https://github.com/UBCMOCCA/TerrainRLSim/tree/master/data/characters))
* Added a small visualizer/editor? (will add edition functionality after some extra changes)
* ... (will check commits later)

### 17, feb, 2019

* Added dynamic loading functionality: We can load any simulation and visualizer at runtime,
    which will be used later when choosing among various physics backends. See test_runtime.cpp 
    in the examples folder. In the example we only change a line to indicate which visualizer, and
    voila, you just swapped the visualizer, no need to recompile. I will be fixing the other examples
    tomorrow (I kind of leave the old examples unusable).

![tysoc-mjc-progress-dlloading-custom-viz](https://media.giphy.com/media/nmtB8f24YJmEXz3KU6/giphy.gif)

![tysoc-mjc-progress-dlloading-mujoco-viz](https://media.giphy.com/media/wq8M5dq2ly37c3uZE5/giphy.gif)

* Will be working on the python bindings now, so in the next days we will have a usable
  python simulator. It will be better when we start adding the other backends. First Bullet,
  and then PhysX.

### 15, aug, 2019

OK, I've been working in the other repos a lot, and also in this one but hadn't had the chance to
actually update this .md. I guess I will start checking in again to keep track of things.

* I'm currently working on the reset functionality. I thought a while ago that I might have to support
  only hard-resets, meaning to destroy and create the simulation again, which was weird, as the other
  suites could write directly to the qpos and be done with it. For their case, they just did this because
  they only had one model, or because the models that had valid qpos, and terrain features are static, so
  there might be no effect in changing those. Unfortunately for our case, I needed a bit more control, but
  luckily it's possible to have more detailed control over our wrapped agents in order to reset what we
  want and how we want to (we could even modify the kinematic tree structure if we wanted to).

* I'm currently porting the test functionality I made for mujoco cases and implementing the reset functionality.
  Also, we could just make some simple functionality to sample from an intitial distribution, which
  the user might want to edit.

* The code is still a mess :/. I've already changed the dependency structure a bit to avoid recursive 
  submodule. It took ages to grab all content with `clone --recursive` as I couldn't figure out how to 
  filter out which submodules would skip downloading its own submodules (nested submodules :( )

* In regards of the codebase, I'm trying to refactor some things and getting rid of code that is redundant
  (like the mujoco-viz, and the bodies/joints/... sandbox-primitives). So, things might break a lot today
  XD. Hopefully, still nobody uses this repo but myself, but will have to create dev branches just in
  case to avoid these issues in the fugure.

TODOs:

* Refactor code a bit to remove redundant functionality
* Implement reset functionality (port from tests)
* Test/debug reset functionality in both C++ and Python APIs
* Initialize documentation for this repo, as the core repo
  already has some started sphinx configuration for its docs.