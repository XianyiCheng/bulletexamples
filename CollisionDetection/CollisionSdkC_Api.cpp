#include "CollisionSdkC_Api.h"
#include "CollisionSdkInterface.h"
#include "Bullet2CollisionSdk.h"

/* Collision World */

plCollisionWorldHandle plCreateCollisionWorld(plCollisionSdkHandle collisionSdkHandle, int maxNumObjsCapacity, int maxNumShapesCapacity, int maxNumPairsCapacity)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createCollisionWorld(maxNumObjsCapacity, maxNumShapesCapacity, maxNumPairsCapacity);
}

void plDeleteCollisionWorld(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	if (sdk && worldHandle)
	{
		sdk->deleteCollisionWorld(worldHandle);
	}
}

plCollisionSdkHandle plCreateBullet2CollisionSdk()
{
	return Bullet2CollisionSdk::createBullet2SdkHandle();
}


void plDeleteCollisionSdk(plCollisionSdkHandle collisionSdkHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	delete sdk;
}

plCollisionShapeHandle plCreateSphereShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plReal radius)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createSphereShape(worldHandle, radius);
}

plCollisionShapeHandle plCreatePlaneShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle,
										  plReal planeNormalX,
										  plReal planeNormalY,
										  plReal planeNormalZ,
										  plReal planeConstant)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createPlaneShape(worldHandle, planeNormalX, planeNormalY, planeNormalZ, planeConstant);
}

plCollisionShapeHandle plCreateCapsuleShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plReal radius, plReal height, int capsuleAxis)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createCapsuleShape(worldHandle, radius, height, capsuleAxis);
}

plCollisionShapeHandle plCreateBoxShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plReal half_length, plReal half_width, plReal half_height)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createBoxShape(worldHandle, half_length, half_width, half_height);
}


plCollisionShapeHandle plCreateCompoundShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createCompoundShape(worldHandle);
}
void plAddChildShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plCollisionShapeHandle compoundShape, plCollisionShapeHandle childShape, plVector3 childPos, plQuaternion childOrn)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->addChildShape(worldHandle, compoundShape, childShape, childPos, childOrn);
}

void plDeleteShape(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plCollisionShapeHandle shapeHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->deleteShape(worldHandle, shapeHandle);
}

plCollisionObjectHandle plCreateCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, void* userData, int userIndex, plCollisionShapeHandle cshape, plVector3 childPos, plQuaternion childOrn)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->createCollisionObject(worldHandle, userData, userIndex, cshape, childPos, childOrn);
}

void plDeleteCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plCollisionObjectHandle body)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->deleteCollisionObject(body);
}

void plSetCollisionObjectTransform(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plCollisionObjectHandle objHandle, plVector3 position, plQuaternion orientation)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->setCollisionObjectTransform(worldHandle, objHandle, position, orientation);
}

void plAddCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle world, plCollisionObjectHandle object)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->addCollisionObject(world, object);
}
void plRemoveCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle world, plCollisionObjectHandle object)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->removeCollisionObject(world, object);
}

/* Collision Queries */
int plCollide(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plCollisionObjectHandle colA, plCollisionObjectHandle colB,
			  lwContactPoint* pointsOut, int pointCapacity)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	return sdk->collide(worldHandle, colA, colB, pointsOut, pointCapacity);
}

void plWorldCollide(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle world,
					plNearCallback filter, void* userData)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*)collisionSdkHandle;
	sdk->collideWorld(world, filter, userData);
}
