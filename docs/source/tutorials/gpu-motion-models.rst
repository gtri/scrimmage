.. _clinfo: https://github.com/Oblomov/clinfo


GPU Motion Updates
===================================

TLDR;
___________________________________
To use a GPU motion kernel in SCRIMMAGE:

.. code-block:: xml

  <gpu_kernel 
  src_dirs = "SimpleAircraft" 
  include_dirs = "kernels"
  kernel_name = "SimpleAircraft">SimpleAircraftDouble
  </gpu_kernel>

  <entity>
    ...
    <gpu_motion_model>SimpleAircraftDouble</gpu_motion_model>
    ...
  </entity>

Motivation:
______________________________________



Declaring Kernels in a Mission File:
___________________________________

SCRIMMAGE is informed of existing kernels through the '<gpu_kernel>' xml tag. Much like C source code, 
gpu kernels must be compiled for the various devices they're intended to execute on. 
The following attributes are used in specifing the kernel compilation:

| Attribute | Values | Required | Description | 
|-----------|--------|----------|-------|
| `src_dirs`  | "src_dir1,src_dir2,..."| YES | A comma seperated list of values. If the location is not explicitly 
specified, SCRIMMAGE will recursivly search `SCRIMMAGE_KERNEL_DIR` until a matching directory is found. All files with 
a `.cl` extension are considered source files and are included in the compilation.|
| `kernel_name`  | "kernel_name" | YES | The name of OpenCL kernel function SCRIMMAGE will call. |
| `include_dirs` | "include_dir1,include_dir2,..." | NO | Specify the INCLUDE directories where header files are located. Search rules are the same as for `src_dirs` | 
| `single_precision` | "true"/"false" | NO | Specifies if the kernel should represent numbers using Single or Double Floating precision. By default, Double precision is used, unless the compilation target does not support Double precision. |
| `platforms` | "platform1,platform2,..... | NO | OpenCL Platforms to target. If specified, the devices that SCRIMMAGE will select from is reduced to only those belonging to the specified platforms.

For example, the xml tag below defines a gpu-kernel named `SimpleAircraftSingle` which is the Single precision implementation of 
the `SimpleAircraft` kernel, found in `$SCRIMMAGE_KERNEL_DIR/motion/SimpleAircraft/SimpleAircraft.cl` using 
`$SCRIMMAGE_KERNEL_DIR` as an INCLUDE directory. This kernel will target all devices belonging 
to the NVIDIA CUDA platform.


.. code-block:: xml
   :linenos:

  <!-- Kernel Used for Motion Updates -->
  <gpu_kernel 
  src_dirs = "SimpleAircraft" 
  include_dirs = "kernels"
  single_precision="true" 
  platforms = 'NVIDIA CUDA'
  kernel_name = "SimpleAircraft">SimpleAircraftSingle
  </gpu_kernel>


Platforms:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To determine what OpenCL Platforms you have on your device (and/or to test if you installed OpenCL correctly), install and run clinfo_. 

On Linux:
::

  $ sudo apt install clinfo
  $ clinfo


`clinfo` will output information about every OpenCL platform and device on your computer.
If not explicitly provided in the kernel xml tag, all platforms will be considered, and the fastest device found will 
be selected as the target for all kernels. This isn't the perfect way to do this, but provides a good heuristic.

Connecting an Entity with a Motion Kernel
_________________________________________

Giving kernels specific names (e.g. `SimpleAircraftSingle` or `SimpleAircraftDouble`) lets us compile different versions
of the same kernel depending on the accuracy requirements of the mission and/or specific entities.
An entity specifies that it uses a kernel for motion updates with the `<gpu_motion_model>` tag,
which replaces the normal `<motion_model>` tag. For example, an entity that 
uses the `SimpleAircraftSingle` motion kernel would have the following tags:

.. code-block:: xml
  <entity>
    <team_id>1</team_id>
    ...
    <controller>SimpleAircraftControllerPID</controller> 
    <gpu_motion_model>SimpleAircraftSingle</gpu_motion_model>
    ...
  </entity>

Only one `<gpu_motion_model>` tag may be used per entity.


Writing Kernels
________________________________

In principal, the GPU motion models are similar to their CPU-based counterparts: Every timestep
a set of differential equations are integrated to produce the entities state for the next timestep/simulation loop.

However, in practice there are quite a few differences.   


Examine SimpleAircraft Kernel
---------------------------------

