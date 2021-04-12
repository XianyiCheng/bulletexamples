# HelloWorld is a minimal sample creating, stepping and deleting a Bullet dynamics world

cmake_minimum_required(VERSION 2.4.3)


# set the project name
project(App_HelloWorld)

set(BULLET_PHYSICS_SOURCE_DIR "/home/xianyi/libraries/bullet3")
set(BULLET_PHYSICS_LIB_DIR "/home/xianyi/libraries/bullet3/build3")
include_directories(${BULLET_PHYSICS_SOURCE_DIR}/src)
link_directories(
${BULLET_PHYSICS_LIB_DIR}/src/Bullet3Common
${BULLET_PHYSICS_LIB_DIR}/src/Bullet3Dynamics
${BULLET_PHYSICS_LIB_DIR}/src/Bullet3Geometry
${BULLET_PHYSICS_LIB_DIR}/src/Bullet3OpenCL
${BULLET_PHYSICS_LIB_DIR}/src/Bullet3OpenCL_clew
${BULLET_PHYSICS_LIB_DIR}/src/BulletCollision
${BULLET_PHYSICS_LIB_DIR}/src/BulletDynamics
${BULLET_PHYSICS_LIB_DIR}/src/BulletInverseDynamics
${BULLET_PHYSICS_LIB_DIR}/src/BulletSoftBody
${BULLET_PHYSICS_LIB_DIR}/src/LinearMath
)

LINK_LIBRARIES(BulletDynamics BulletCollision LinearMath)


ADD_EXECUTABLE(App_HelloWorld HelloWorld.cpp)


