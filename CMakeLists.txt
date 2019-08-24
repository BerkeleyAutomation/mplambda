# 3.10 introduces C++ 17 support
cmake_minimum_required (VERSION 3.10)
project (ur5)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable extensions to keep compatible with standards
set(CMAKE_CXX_EXTENSIONS OFF)

## find_package(FCL REQUIRED)
######## Find CCD the hard way
find_package(PkgConfig)
if (PKGCONFIG_FOUND)
    pkg_check_modules(FCL fcl>=0.6)
    pkg_check_modules(CCD ccd>=2.0)
else()
    find_package(FCL REQUIRED)
    find_package(CCD CONFIG)
endif()
#########

find_package(OpenMP REQUIRED)
find_package(Assimp REQUIRED)
find_package(Threads)
find_package(Eigen3 3.3 REQUIRED NO_MODULE) 

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(${FCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})
include_directories(include ../jilog/include ../nigh/src ../bump/include ${FCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS})

add_executable(mpl_prrt_or src/mpl_prrt_or.cpp)
target_link_libraries(mpl_prrt_or Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})
