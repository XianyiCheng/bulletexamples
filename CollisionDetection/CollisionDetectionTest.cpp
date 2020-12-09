#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "CollisionSdkC_Api.h"
#include <stdio.h>
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "LinearMath/btQuickprof.h"

static int gTotalPoints = 0;
const int sPointCapacity = 50;

const int sNumSpheres = 3;

lwContactPoint pointsOut[sPointCapacity];
int numNearCallbacks = 0;

void myNearCallback(plCollisionSdkHandle sdkHandle, plCollisionWorldHandle worldHandle, void* userData, plCollisionObjectHandle objA, plCollisionObjectHandle objB)
{
	numNearCallbacks++;
	int remainingCapacity = sPointCapacity - gTotalPoints;
	btAssert(remainingCapacity > 0);

	if (remainingCapacity > 0)
	{
		lwContactPoint* pointPtr = &pointsOut[gTotalPoints];
		int numNewPoints = plCollide(sdkHandle, worldHandle, objA, objB, pointPtr, remainingCapacity);
		btAssert(numNewPoints <= remainingCapacity);
		gTotalPoints += numNewPoints;
	}
}

int main(int argc, char** argv)
{

    plCollisionSdkHandle m_collisionSdkHandle = plCreateBullet2CollisionSdk();

    int maxNumObjsCapacity = 1024;
    int maxNumShapesCapacity = 1024;
    int maxNumPairsCapacity = 16384;
    btAlignedObjectArray<plCollisionObjectHandle> colliders;
    plCollisionWorldHandle m_collisionWorldHandle = plCreateCollisionWorld(m_collisionSdkHandle, maxNumObjsCapacity, maxNumShapesCapacity, maxNumPairsCapacity);

    //create objects, do query etc
    {
        float radius = 1.f;

        void* userPointer = 0;

        {
            plCollisionShapeHandle colShape = plCreateBoxShape(m_collisionSdkHandle, m_collisionWorldHandle, 0.5,0.5,0.5);
            btVector3 pos(-2, -0.405, 0);
            btQuaternion orn(0, 0, 0, 1);
            void* userPointer = 0;
            plCollisionObjectHandle colObj = plCreateCollisionObject(m_collisionSdkHandle, m_collisionWorldHandle, userPointer, -1, colShape, pos, orn);
            colliders.push_back(colObj);
            plAddCollisionObject(m_collisionSdkHandle, m_collisionWorldHandle, colObj);
        }

        {
            // create compound shape
            plCollisionShapeHandle compoundShape = plCreateCompoundShape(m_collisionSdkHandle, m_collisionWorldHandle);
            for (int i = 0; i < sNumSpheres; i++)
            {
                btVector3 childPos(i * 1.5, 0, 0);
                btQuaternion childOrn(0, 0, 0, 1);

                plCollisionShapeHandle childShape = plCreateSphereShape(m_collisionSdkHandle, m_collisionWorldHandle, radius);
                plAddChildShape(m_collisionSdkHandle, m_collisionWorldHandle, compoundShape, childShape, childPos, childOrn);

            }

            btVector3 pos(0 * sNumSpheres * 1.5, 0, 0);
            btQuaternion orn(0, 0, 0, 1);
            plCollisionObjectHandle colObjHandle = plCreateCollisionObject(m_collisionSdkHandle, m_collisionWorldHandle, userPointer, -1, compoundShape, pos, orn);

            colliders.push_back(colObjHandle);
            plAddCollisionObject(m_collisionSdkHandle, m_collisionWorldHandle, colObjHandle);

        }
    }

    {
        plCollisionShapeHandle colShape = plCreatePlaneShape(m_collisionSdkHandle, m_collisionWorldHandle, 0, 1, 0, -0.9);
        btVector3 pos(0, 0, 0);
        btQuaternion orn(0, 0, 0, 1);
        void* userPointer = 0;
        plCollisionObjectHandle colObj = plCreateCollisionObject(m_collisionSdkHandle, m_collisionWorldHandle, userPointer, 0, colShape, pos, orn);
        colliders.push_back(colObj);
        plAddCollisionObject(m_collisionSdkHandle, m_collisionWorldHandle, colObj);
    }

    int numContacts = plCollide(m_collisionSdkHandle, m_collisionWorldHandle, colliders[0], colliders[2], pointsOut, sPointCapacity);
    printf("numContacts = %d\n", numContacts);
    printf("Contact distances: ");
    for (int k = 0; k < numContacts; k++){
        printf(" %f, ", pointsOut[k].m_distance);
    }
    printf("\n");
    void* myUserPtr = 0;

    plWorldCollide(m_collisionSdkHandle, m_collisionWorldHandle, myNearCallback, myUserPtr);
    printf("total points=%d\n", gTotalPoints);

    plDeleteCollisionWorld(m_collisionSdkHandle, m_collisionWorldHandle);

    plDeleteCollisionSdk(m_collisionSdkHandle);

}