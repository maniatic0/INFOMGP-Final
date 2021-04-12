![exploding](https://user-images.githubusercontent.com/72574257/114452516-d133b480-9bd8-11eb-996f-5461fbcff176.png)

# Final project Christian and Tom

## Date: 13/Apr/2021.

The project was built upon the second practical from the course.


## Installation

For successful compilation, it is necessary to have the latest version of boost (https://www.boost.org/users/download/) installed on your root directory, e.g. c:\boost\. Note that the boost directory should be renamed to just 'boost', without the version numbering.

*The rest of the installation is exactly like that of Practical 2, repeated here for completeness*

The skeleton uses the following dependencies: [libigl](http://libigl.github.io/libigl/), and consequently [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), for the representation and viewing of geometry, and [libccd](https://github.com/danfis/libccd) for collision detection. libigl viewer is using [dear imGui](https://github.com/ocornut/imgui) for the menu. Everything is bundled as either submodules, or just incorporated code within the environment, and you do not have to take care of any installation details. To get the library, use:

```bash
git clone --recursive https://github.com/maniatic0/INFOMGP-Final.git
```

to compile the environment, go into the `final` folder and enter in a terminal (macOS/Linux):

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

In windows, you need to use [cmake-gui](https://cmake.org/runningcmake/). Pressing twice ``configure`` and then ``generate`` will generate a Visual Studio solution in which you can work. The active soution should be ``final_bin``. *Note*: it only seems to work in 64-bit mode. 32-bit mode might give alignment errors.



### Input

The TXT file that describes the scene, where you have several examples in the`data` subfolder, is the same. For completeness, the format of the file is:

```
#num_objects
object1.mesh  density1  youngModulus1 PoissonRatio1 is_fixed1    COM1     q1
object2.mesh  density2  youngModulus2 PoissonRatio2 is_fixed2    COM2     q2
.....
```

Where:

1. ``objectX.mesh`` - an MESH file (automatically assumed in the `data` subfolder) describing the geometry of a tetrahedral mesh. The original coordinates are translated automatically to have $(0,0,0)$ as their COM.
<br />
2. ``density`` - the uniform density of the object. The program will automatically compute the total mass by the volume.
<br />
3. ``is_fixed`` - if the object should be immobile (fixed in space) or not.
<br />
4. ``COM`` - the initial position in the world where the object would be translated to. That means, where the COM is at time $t=0$.
<br />
5. ``q`` - the initial orientation of the object, expressed as a quaternion that rotates the geometry to $q*object*q^{-1}$ at time $t=0$.
<br />
6. ``youngModulus1`` and  ``PoissonRatio1`` should be ignored for now; we will use them in the $3^{rd}$ practical.

The user attachment constraints file, given as the third argument, has to have the following format:

```
#num_constraints
mesh_i1 vertex_i1 mesh_j1 vertex_j1 
mesh_i2 vertex_i2 mesh_j2 vertex_j2 
.....
```

Each row is a constraint attaching the vertex ```vertex_i1``` of mesh ```mesh_i1``` to ```vertex_j2``` of mesh ```mesh_j2```. Every row is an independent such constraint. You can find TXT files in the data folder with similar name to the scenes they accompany. You can of course write new ones. Note that the meshes start indexing from $1$---if you put a constraint to mesh $0$, it will get attached to the platform (which should still work).

