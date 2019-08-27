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
* (Optional: ninja [https://ninja-build.org/] build tool, usually in package systems)

# Setup instructions

First install required software.  Note that `nigh` should be checked out to the same level as this project.  For example, if you have `mplambda` checked out to `~/projects/mplambda`, nigh should be checked out to `~/projects/nigh`

To build:

```console
# change to the checked out directory
$ cd mplambda
$ mkdir -p build/Debug
$ cd build/Debug
```

(Option 1) for ninja builds, run:

```console
$ cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja ../..
$ ninja
```

(Option 2) for make builds, run:

```console
$ cmake -DCMAKE_BUILD_TYPE=Debug ../..
$ make -j4
```

# Troubleshooting

If your C++17 compiler has a different name than the default one used by CMake, you can specify which compiler to use with the `CXX` environment variable, e.g., the following example uses `clang++-mp-7.0` as the CXX compiler.

```console
$ CXX=clang++-mp-7.0 cmake -DCMAKE_BUILD_TYPE=Debug ../..
```

If CMake complains about pkgconfig, then you may have to set the `PKG_CONFIG_PATH` environment variable.  This can be done in the same way as `CXX`.  Typically, this message can be fixed by adding `/usr/local/lib/pkgconfig`.
```console
$ PKG_CONFIG_PATH=/usr/local/lib/pkgconfig cmake -DCMAKE_BUILD_TYPE=Debug ../..
```

To specify both `PKG_CONFIG_PATH` and `CXX`, separate them by a space.
