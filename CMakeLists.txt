cmake_minimum_required (VERSION 3.8) 
project(imu_tk_ros2)

cmake_policy(SET CMP0015 NEW)

set(CMAKE_BUILD_TYPE "Debug")

if(NOT DEFINED BUILD_IMU_TK_EXAMPLES)
  set(BUILD_IMU_TK_EXAMPLES "ON")
endif(NOT DEFINED BUILD_IMU_TK_EXAMPLES)

find_package(Boost REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(/usr/include/eigen3)
include_directories(./include
                    /usr/include
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})

#Vis3D
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS}  
                     ${GLUT_INCLUDE_DIRS}
                     ${CMAKE_CURRENT_BINARY_DIR} )
                     
set(IMU_TK_VIS_EXTRA_SOURCES 
    ./src/vis_extra/gl_camera.cpp 
    ./src/vis_extra/opengl_3d_scene.cpp )
set(IMU_TK_VIS_EXTRA_HEADERS
    ./include/imu_tk/vis_extra/opengl_3d_scene.h )
    
qt5_wrap_cpp(IMU_TK_VIS_EXTRA_HEADERS_MOC ${IMU_TK_VIS_EXTRA_HEADERS})

# include(${QT_USE_FILE})
# add_definitions(${QT_DEFINITIONS})

aux_source_directory( ./src IMU_TK_CORE_SRC )
set( IMU_TK_SRC ${IMU_TK_CORE_SRC} ${IMU_TK_VIS_EXTRA_HEADERS_MOC} ${IMU_TK_VIS_EXTRA_SOURCES} )

add_library(imu_tk ${IMU_TK_SRC})
target_link_libraries(imu_tk Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
set_target_properties(imu_tk PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set (IMU_TK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "imu_tk include directories")
set (IMU_TK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib CACHE STRING "imu_tk libraries directories")
set (IMU_TK_LIBS imu_tk ${CERES_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} 
     CACHE STRING "imu_tk libraries")

message( "${IMU_TK_LIBS}" )


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(${IMU_TK_INCLUDE_DIR})

ament_auto_add_executable(imu_tk_node apps/imu_calib_ros2.cpp)
target_link_libraries(imu_tk_node ${IMU_TK_LIBS})

ament_auto_package(INSTALL_TO_SHARE
  launch)
