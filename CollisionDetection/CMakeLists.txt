cmake_minimum_required(VERSION 2.4.3)

# set the project name
project(App_Collision_Detection)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -std=gnu++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=gnu++0x")


OPTION(BUILD_ENET "Set when you want to build apps with enet UDP networking support" OFF)
OPTION(BUILD_CLSOCKET "Set when you want to build apps with enet TCP networking support" OFF)


set(BULLET_PHYSICS_SOURCE_DIR "/home/xianyi/libraries/bullet3")

INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Gwen
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples/CommonInterfaces
		${BULLET_PHYSICS_SOURCE_DIR}/examples/Utils
		${BULLET_PHYSICS_SOURCE_DIR}/examples/OpenGLWindow
		${BULLET_PHYSICS_SOURCE_DIR}/examples/BulletRobotics
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser
		${BULLET_PHYSICS_SOURCE_DIR}/Extras
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/InverseDynamics
		${BULLET_PHYSICS_SOURCE_DIR}/bin
  )
  
link_directories( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/src/Bullet3Common
		${BULLET_PHYSICS_SOURCE_DIR}/src/Bullet3Dynamics
		${BULLET_PHYSICS_SOURCE_DIR}/src/Bullet3Geometry
		${BULLET_PHYSICS_SOURCE_DIR}/src/Bullet3OpenCL
		${BULLET_PHYSICS_SOURCE_DIR}/src/Bullet3OpenCL_clew
		${BULLET_PHYSICS_SOURCE_DIR}/src/BulletCollision
		${BULLET_PHYSICS_SOURCE_DIR}/src/BulletDynamics
		${BULLET_PHYSICS_SOURCE_DIR}/src/BulletInverseDynamics
		${BULLET_PHYSICS_SOURCE_DIR}/src/BulletSoftBody
		${BULLET_PHYSICS_SOURCE_DIR}/src/LinearMath
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Gwen
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples/CommonInterfaces
		${BULLET_PHYSICS_SOURCE_DIR}/examples/Utils
		${BULLET_PHYSICS_SOURCE_DIR}/examples/OpenGLWindow
		${BULLET_PHYSICS_SOURCE_DIR}/examples/BulletRobotics
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser
		${BULLET_PHYSICS_SOURCE_DIR}/Extras
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/InverseDynamics
		${BULLET_PHYSICS_SOURCE_DIR}/Extras/BulletRobotics
		${BULLET_PHYSICS_SOURCE_DIR}/bin
  )
  
LINK_LIBRARIES(BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common -ldl)

#add_definitions     ( ${BULLET_DEFINITIONS} )
#include_directories ( ${BULLET_INCLUDE_DIRS} )
#link_directories    ( ${BULLET_LIBRARY_DIRS} )


#some code to support OpenGL and Glew cross platform
IF (WIN32)
	INCLUDE_DIRECTORIES(
			${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
	IF(APPLE)
		find_library(COCOA NAMES Cocoa)
		MESSAGE(${COCOA})
		link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

	ELSE(APPLE)
		INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
		)
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
		ADD_DEFINITIONS("-DGLEW_STATIC")
		ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

		LINK_LIBRARIES(  pthread ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)

IF(NOT WIN32)
	SET(DL ${CMAKE_DL_LIBS})
	IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
		MESSAGE("Linux")
		SET(OSDEF -D_LINUX)
	ELSE(CMAKE_SYSTEM_NAME MATCHES "Linux")
		IF(APPLE)
			MESSAGE("Apple")
			SET(OSDEF -D_DARWIN)
		ELSE(APPLE)
			MESSAGE("BSD?")
			SET(OSDEF -D_BSD)
		ENDIF(APPLE)
	ENDIF(CMAKE_SYSTEM_NAME MATCHES "Linux")
ENDIF(NOT WIN32)

IF(NOT WIN32 AND NOT APPLE)
    OPTION(BUILD_EGL "Build OpenGL/EGL" ON)
    IF(BUILD_EGL)
        ADD_DEFINITIONS(-DBT_USE_EGL)
    ENDIF(BUILD_EGL)
ENDIF()

SET(CollisionDetection_SRCS
	CollisionDetectionTest.cpp
	Bullet2CollisionSdk.cpp
	Bullet2CollisionSdk.h
	CollisionSdkC_Api.cpp
	CollisionSdkC_Api.h
	CollisionSdkInterface.h
)	

#ADD_EXECUTABLE(App_CollisionDetection  ${CollisionDetection_SRCS})
ADD_EXECUTABLE(App_CollisionDetection  SimpleCollisionDetection.cpp)

if(FALSE) # fake a block comment

SET(CollisionDetection_GUI_SRCS
	CollisionDetectionTest_GUI.cpp
	Bullet2CollisionSdk.cpp
	Bullet2CollisionSdk.h
	CollisionSdkC_Api.cpp
	CollisionSdkC_Api.h
	CollisionSdkInterface.h
)	

ADD_EXECUTABLE(App_CollisionDetection_GUI  ${CollisionDetection_GUI_SRCS})
endif()