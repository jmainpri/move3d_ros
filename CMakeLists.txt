cmake_minimum_required(VERSION 2.8.3)
project(move3d_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  pr2_controllers_msgs
  actionlib
  control_msgs
  sensor_msgs
)

find_package(cmake_modules REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs#   trajectory_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS control_msgs roscpp rospy std_msgs trajectory_msgs sensor_msgs actionlib DEPENDS system_lib Eigen)

###########
## Build ##
###########



#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# --- Compilation Flags --------------------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

SET(Move3D_CompilationFlags "")
SET(Move3D_LinkedCompilationFlags "")

MACRO(Move3D_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND Move3D_CompilationFlags ${flagName})
  FOREACH(linkFlag ${ARGN})
    LIST(APPEND Move3D_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(Move3D_flag)

MACRO(Move3D_CheckCompilationFlags)
  SET(Move3D_ActiveFlags "")
  #MESSAGE ("Active Flags :")
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "Move3D_COMPILATION_FLAGS = ")
  FOREACH(flag ${Move3D_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/Move3DFlags "-D${flag} ")
      #MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND Move3D_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)
ENDMACRO(Move3D_CheckCompilationFlags)

Move3D_flag( QT_LIBRARY "Sets QtLibrary compiling mode" ON)
Move3D_flag( QT_GL "Sets QtLibrary compiling mode" ON QT_LIBRARY)
Move3D_flag( QT_UI_XML_FILES "Sets QtLibrary compiling mode" ON QT_LIBRARY)

Move3D_flag( HRI_COSTSPACE "With the hri work" ON)


Move3D_flag( P3D_CONSTRAINTS "Compilation with module Constraints" ON)
Move3D_flag( P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
Move3D_flag( P3D_LOCALPATH "Compilation with module Localpath" ON)
Move3D_flag( P3D_PLANNER "Compilation with module Planner" ON)
Move3D_flag( LIGHT_PLANNER "Compilation with module Light Planner" ON)
Move3D_flag( GRASP_PLANNING "Compilation with module Manipulation Planner" ON)
Move3D_flag( MULTILOCALPATH "Compilation with module Multilocalpath" ON)
Move3D_flag( MULTIGRAPH "Compilation with module Multi-Graph" OFF)

Move3D_flag( MOVE3D_CORE "Compiles the planning API and planners" ON)

Move3D_flag( HRI_PLANNER "Human Robot Interaction planner module" ON USE_GSL PQP)

Move3D_flag( USE_GBM "Compilation with module Multilocalpath" OFF)
Move3D_flag( USE_GLUT "Compilation with a glut window" OFF)
Move3D_flag( USE_GSL "Mathematic Library" OFF)
Move3D_flag( USE_QWT "Compilation with technical widget" ON)

Move3D_flag( PQP "Compilation with technical widget" ON)

# Find important packages
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

#-------------- Eigen ----------------------------------------------------------

find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
message(Eigen_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS})

#-------------- Boost ----------------------------------------------------------

# boost is used e.g. for managing the cost functions (T-RRT).
#  FIND_PACKAGE(Boost REQUIRED)
#  INCLUDE_directories(${BOOST_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${BOOST_INCLUDE_DIR})

# boost is used e.g. for managing the cost functions (T-RRT).
find_package(Boost REQUIRED)
include_directories(/opt/local/include/)
set(INCLUDE_DIRS ${INCLUDE_DIRS} /opt/local/include/ ${BOOST_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#-------------- LibXml2 --------------------------------------------------------

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

# ------------------------------------------------------------------------------
#-------------- Qt Library -----------------------------------------------------
# ------------------------------------------------------------------------------

IF(QT_LIBRARY)
  # This also fails if qt4 is installed, but system is configured for qt3 qmake
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(Move3DGUI_COMPILATION_INCLUDES ${Move3DGUI_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})
  IF(QT_GL)
    SET(QT_USE_QTOPENGL TRUE)
  ENDIF(QT_GL)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})
ENDIF(QT_LIBRARY)

find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${X11_INCLUDE_DIR})
SET(LIBS ${LIBS} ${X11_LIBRARIES})

IF(APPLE)
        IF(QT_LIBRARY)
                FIND_PACKAGE(OpenGL REQUIRED)
                INCLUDE_directories("${OPENGL_INCLUDE_DIR}/Headers")
                SET(INCLUDE_DIRS ${INCLUDE_DIRS} "${OPENGL_INCLUDE_DIR}/Headers")
                SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
                #Message(${OPENGL_INCLUDE_DIR})
                #Message(${OPENGL_LIBRARIES})
                IF(WITH_XFORMS)
                        FIND_PACKAGE(GLU REQUIRED)
                        INCLUDE_directories(${GLU_INCLUDE_DIR})
                        SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
                        SET(LIBS ${LIBS} ${GLU_LIBRARIES})
                        FIND_PACKAGE(GL REQUIRED)
                        INCLUDE_directories(${GL_INCLUDE_DIR})
                        SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
                        SET(LIBS ${LIBS} ${GL_LIBRARIES})
                ENDIF(WITH_XFORMS)
        ENDIF(QT_LIBRARY)
ELSE(APPLE)
  find_package(OpenGL REQUIRED)
  include_directories(${OPENGL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
ENDIF(APPLE)

IF(QT_LIBRARY)

  # generate the UI headers
  QT4_WRAP_UI(Move3DGUI_QT_UI_HEADERS include/move3d_ros_gui.ui)

  message(${Move3DGUI_QT_UI_HEADERS})

  # Qt4 files that need to be preprocessed
  QT4_WRAP_CPP(Move3DGUI_QT_MOC include/move3d_ros_node.hpp 
                                include/move3d_ros_gui.hpp 
                                include/move3d_ros_human.hpp
                                include/move3d_ros_replanning.hpp
                                include/move3d_ros_robot.hpp)


ENDIF(QT_LIBRARY)

# ----------------------------------------------------------------------------
# ------------------- libmove3d ----------------------------------------------
# ----------------------------------------------------------------------------

find_package (PkgConfig REQUIRED)

pkg_check_modules(MOVE3D REQUIRED libmove3d)
include_directories(${MOVE3D_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
ADD_DEFINITIONS(${MOVE3D_CFLAGS})

message("MOVE3D_INCLUDE_DIRS : ${MOVE3D_INCLUDE_DIRS}")
message("MOVE3D_LIBRARIES : ${MOVE3D_LIBRARIES}")
message("MOVE3D_CFLAGS : ${MOVE3D_CFLAGS}")

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX -x c++)
    SET (CMAKE_SHARED_LINKER_FLAGS "-dynamiclib -Wl,-headerpad_max_install_names,-undefined,dynamic_lookup")
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

#MESSAGE( MOVE3D_LIBRARIES = ${MOVE3D_LIBRARIES})


pkg_check_modules(MOVE3D-PLANNERS REQUIRED libmove3d-planners)
include_directories(${MOVE3D-PLANNERS_INCLUDE_DIRS})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-PLANNERS_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${MOVE3D-PLANNERS_LIBRARIES})
ADD_DEFINITIONS(${MOVE3D-PLANNERS_CFLAGS})

message("MOVE3D-PLANNERS_INCLUDE_DIRS : ${MOVE3D-PLANNERS_INCLUDE_DIRS}")
message("MOVE3D-PLANNERS_LIBRARIES : ${MOVE3D-PLANNERS_LIBRARIES}")

pkg_check_modules(MOVE3D-QTGUI REQUIRED move3d-qt-gui-libs)
INCLUDE_directories(${MOVE3D-QTGUI_INCLUDE_DIRS})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-QTGUI_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${MOVE3D-QTGUI_LIBRARIES})
ADD_DEFINITIONS(${MOVE3D-QTGUI_CFLAGS})

message("MOVE3D-QTGUI_DIRS : ${MOVE3D-QTGUI_INCLUDE_DIRS}")
message("MOVE3D-QTGUI_LIBRARIES : ${MOVE3D-QTGUI_LIBRARIES}")


IF( MULTILOCALPATH )

    #-------------- gbM -----------------
    pkg_check_modules(GBM REQUIRED gbM)
    include_directories(${GBM_INCLUDE_DIR})
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
    SET(LIBS ${LIBS} ${GBM_LIBRARIES})

    #-------------- softMotion -----------
    pkg_check_modules(SOFTMOTION REQUIRED softMotion-libs)
    include_directories(${SOFTMOTION_INCLUDE_DIR})
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${SOFTMOTION_INCLUDE_DIRS})
    SET(LIBS ${LIBS} ${SOFTMOTION_LIBRARIES})

ENDIF()

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include)
include_directories($ENV{HOME}/catkin_ws/build/move3d_ros) # TODO FIX remove

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(move3d_ros src/${PROJECT_NAME}/move3d_ros.cpp )

add_library(${PROJECT_NAME} src/move3d_ros_gui.cpp
                            src/move3d_ros_human.cpp
                            src/move3d_ros_replanning.cpp
                            src/move3d_ros_robot.cpp
                            ${Move3DGUI_QT_MOC} ${Move3DGUI_QT_UI_HEADERS})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBS})

add_executable(${PROJECT_NAME}_node src/move3d_ros_node.cpp ${Move3DGUI_QT_MOC})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} move3d_ros_node
   #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_move3d_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
