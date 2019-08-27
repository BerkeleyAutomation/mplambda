# mplambda
Motion Planning Lambdas

# Requirements
* CMake 3.11+ [https://cmake.org/]
* C++ 17 compiler (GCC 8+ or clang 5+ recommended) 
* nigh [https://github.com/UNC-Robotics/nigh]
* Eigen [http://eigen.tuxfamily.org/index.php?title=Main_Page]
* FCL [https://github.com/flexible-collision-library/fcl] (build from source)
* libccd [https://github.com/danfis/libccd] (build from source)
* assimp [https://github.com/assimp/assimp] (usually available in package system)
* (Optional: ninja build tool)

# Setup instructions

First install required software.  Note that `nigh` should be checked out to the same level as this project.  For example, if you have `mplambda` checked out to `~/projects/mplambda`, nigh should be checked out to `~/projects/nigh`

To build:

    # change to the checked out directory
    cd mplambda
    mkdir -p build/Debug
    cd build/Debug
    
For ninja builds, then run:

    cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja ../..
    ninja
    
For make builds, run:

    cmake -DCMAKE_BUILD_TYPE=Debug ../..
    make -j4

