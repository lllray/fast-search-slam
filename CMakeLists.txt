cmake_minimum_required(VERSION 2.6)
project(icp_bnb)

SET(CMAKE_BUILD_TYPE Release)
#IF(CMAKE_COMPILER_IS_GNUCC)
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#ELSE()
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#ENDIF()
add_compile_options(-std=c++14)
set(OpenCV_DIR /home/ray/APP/opencv/opencv-3.2.0/release/)
if(POLICY CMP0074)
cmake_policy(SET CMP0074 NEW)
endif()
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  OpenCV REQUIRED
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  message_generation
  tf
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/ray/Tools/g2o/cmake_modules")
#set(G2O_ROOT /home/ray/Tools/g2o)
find_package(CSparse REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)

catkin_package()

IF(G2O_FOUND)
	include_directories(${G2O_INCLUDE_DIR})
	message("G2O lib is found:"${G2O_INCLUDE_DIR})
ENDIF(G2O_FOUND)
include_directories(
	${OpenCV_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
	${CSPARSE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
add_definitions(${EIGEN_DEFINITIONS})


SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )
add_library(${PROJECT_NAME}_lib SHARED
    src/icp.cpp
        include/icp_define.h src/scan_process.cpp include/scan_process.h src/icp_map.cpp include/icp_map.h src/close_loop.cpp include/close_loop.h src/g2o_solver.cpp include/g2o_solver.h src/tools.cpp include/tools.h src/odom_process.cpp include/odom_process.h include/icp_type.h)
target_link_libraries(${PROJECT_NAME}_lib
    ${CSPARSE_LIBRARY}
    ${CHOLMOD_LIBRARIES}
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM2D}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
)

# TEST ICP
add_executable(icp_odometry test/icp_odometry.cpp)
target_link_libraries(icp_odometry ${catkin_LIBRARIES}
		${PROJECT_NAME}_lib
		${G2O_LIBS})