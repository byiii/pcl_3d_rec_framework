cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(3d_rec_framework)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
include (FindPkgConfig)

# eigen
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

# vtk
find_package(VTK REQUIRED)
INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})

# boost
find_package(Boost REQUIRED)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
LINK_DIRECTORIES(${BOOST_LIBRARY_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# flann
find_package(FLANN 1.7.0 REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})

# libusb
find_package(libusb-1.0 REQUIRED)
include_directories("${LIBUSB_1_INCLUDE_DIR}")
link_directories(${LIBUSB_1_LIBRARY_DIR})

# openni
find_package(OpenNI REQUIRED)

# qhull
find_package(Qhull)
include_directories(${QHULL_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(incs_fw "include/feature_wrapper/normal_estimator.h")
set(incs_fw_global   "include/feature_wrapper/global/vfh_estimator.h"
                     "include/feature_wrapper/global/esf_estimator.h"
                     "include/feature_wrapper/global/crh_estimator.h"
                     "include/feature_wrapper/global/global_estimator.h"
                     )
# "include/feature_wrapper/global/cvfh_estimator.h"
#"include/feature_wrapper/global/ourcvfh_estimator.h")

  set(incs_fw_local "include/feature_wrapper/local/local_estimator.h"
                    "include/feature_wrapper/local/fpfh_local_estimator.h"
                    "include/feature_wrapper/local/fpfh_local_estimator_omp.h"
                    "include/feature_wrapper/local/shot_local_estimator.h"
                    "include/feature_wrapper/local/colorshot_local_estimator.h"
                    "include/feature_wrapper/local/shot_local_estimator_omp.h")
  set(incs_pc_source "include/pc_source/source.h"
                     "include/pc_source/mesh_source.h"
                     "include/pc_source/registered_views_source.h")

  set(incs_pipelines "include/pipeline/global_nn_classifier.h"
           "include/pipeline/global_nn_recognizer_crh.h"
           "include/pipeline/local_recognizer.h")

#"include/pipeline/global_nn_recognizer_cvfh.h"

  set(incc_tools_framework "include/tools/openni_frame_source.h"
                           "include/tools/render_views_tesselated_sphere.h")

  set(incs_utils "include/utils/metrics.h"
           "include/utils/persistence_utils.h"
           "include/utils/vtk_model_sampling.h")

  set(srcs src/global_nn_classifier.cpp
           src/global_nn_recognizer_crh.cpp
           src/global_nn_recognizer_cvfh.cpp
           src/local_recognizer.cpp
           src/openni_frame_source.cpp
           src/render_views_tesselated_sphere.cpp)

  set(impl_incs_pipeline "include/pipeline/impl/global_nn_classifier.hpp"
                "include/pipeline/impl/global_nn_recognizer_crh.hpp"
                "include/pipeline/impl/local_recognizer.hpp")
# "include/pipeline/impl/global_nn_recognizer_cvfh.hpp"

  set(LIB_NAME "pcl_apps")
add_library("${LIB_NAME}" ${srcs} ${impl_incs_pipeline} ${incs_utils} ${incs_fw} ${incs_fw_global} ${incs_fw_local} ${incc_tools_framework} ${incs_pipelines} ${incs_pc_source})

 target_link_libraries("${LIB_NAME}"
  ${OPENNI_LIBRARIES} ${LIBUSB_1_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${BOOST_LIBRARIES})


add_executable(pcl_global_classification apps/global_classification.cpp)
target_link_libraries(pcl_global_classification pcl_apps pcl_3d_rec_framework pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface)

add_executable(pcl_local_or_mian apps/local_recognition_mian_dataset.cpp)
target_link_libraries(pcl_local_or_mian pcl_apps pcl_3d_rec_framework pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints)



