# 3.10 introduces C++ 17 support
cmake_minimum_required (VERSION 3.10)
project (mplambda)

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

if (${APPLE})
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAS_AWS_SDK")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_AWS_SDK")
    
    find_package(aws-lambda-runtime REQUIRED)
    find_package(AWSSDK COMPONENTS s3 lambda)
    add_executable(mpl_lambda_aws src/mpl_lambda_aws.cpp src/mpl/demo/lambda_common.cpp src/mpl/comm.cpp src/mpl/write_queue.cpp src/mpl/demo/app_options.cpp)
    target_link_libraries(mpl_lambda_aws PUBLIC Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES} AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES})
    
    # the following line adds "aws-lambda-package-mpl_lambda_aws" as a
    # target which creates a .zip file with everything we need to
    # upload a function to aws.
    aws_lambda_package_target(mpl_lambda_aws)

    # this is the command to run:
    add_custom_target(mpl_lambda_aws_zip
    	COMMAND zip -ur9 mpl_lambda_aws.zip resources
    	DEPENDS aws-lambda-package-mpl_lambda_aws)

    add_executable(mpl_lambda_invoke src/mpl_lambda_invoke.cpp)
    target_link_libraries(mpl_lambda_invoke AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES})

endif()

link_directories(${FCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})
include_directories(include ../nigh/src ${FCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/AUTOLAB.dae
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/resources)
file(GLOB SE3RSRC
     "resources/se3/Twistycool*.dae" "resources/se3/alpha*.dae"
     "resources/se3/Apartment*.dae" "resources/se3/cubicles*.dae"
     "resources/se3/Home*.dae" "resources/se3/Easy*.dae")
file(COPY ${SE3RSRC} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/resources/se3)


add_executable(mpl_lambda_pseudo src/mpl_lambda_pseudo.cpp src/mpl/demo/lambda_common.cpp src/mpl/comm.cpp src/mpl/write_queue.cpp src/mpl/demo/app_options.cpp)
target_link_libraries(mpl_lambda_pseudo Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

add_executable(mpl_coordinator src/mpl_coordinator.cpp src/mpl/write_queue.cpp)
# target_link_libraries(mpl_coordinator Threads::Threads)

add_executable(mpl_robot src/mpl_robot.cpp src/mpl/write_queue.cpp src/mpl/demo/app_options.cpp)

target_link_libraries(mpl_robot Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

