#include "btBulletCollisionCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <stdio.h>

int main(int argc, char** argv)
{

    btAlignedObjectArray<btCollisionObject> colliders;

    // create box shape
	btBoxShape* boxshape = new btBoxShape(btVector3(5,5,5));
    boxshape->setMargin(0.f);

    // create box collision object
    btCollisionObject* box_collision_obj = new btCollisionObject;
    box_collision_obj->setCollisionShape(boxshape);
    btTransform tr_box;
    tr_box.setOrigin(btVector3(0,0,0));
    tr_box.setRotation(btQuaternion(0,0,0,1));
    box_collision_obj->setWorldTransform(tr_box);

    // create box2 shape
	btBoxShape* boxshape2 = new btBoxShape(btVector3(2,2,2));
    boxshape->setMargin(0.f);

    // create box collision object
    btCollisionObject* box_collision_obj_2 = new btCollisionObject;
    box_collision_obj_2->setCollisionShape(boxshape2);
    btTransform tr_box2;
    tr_box2.setOrigin(btVector3(0,5+1.98,0));
    tr_box2.setRotation(btQuaternion(0,0,0,1));
    box_collision_obj_2->setWorldTransform(tr_box2);

    // create plane shape
    btStaticPlaneShape* planeshape = new btStaticPlaneShape(btVector3(0, 1, 0), -5);

    // create plane collision object
    btCollisionObject* plane_collision_obj = new btCollisionObject;
    plane_collision_obj->setCollisionShape(planeshape);

    btTransform tr_plane;
    tr_plane.setOrigin(btVector3(0.1,0,0));
    tr_plane.setRotation(btQuaternion(0,0,0,1));
    plane_collision_obj->setWorldTransform(tr_plane);

    // 
    colliders.push_back(*box_collision_obj);
    colliders.push_back(*box_collision_obj_2);
    colliders.push_back(*plane_collision_obj);

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3	worldAabbMin(-1000,-1000,-1000);
	btVector3	worldAabbMax(1000,1000,1000);

    btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);

    btCollisionWorld* collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);

    for (int i = 0; i < colliders.size(); i++){
        collisionWorld->addCollisionObject(&colliders[i]);
    }

    // all contacts in the world
    collisionWorld->performDiscreteCollisionDetection();

    int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();

    printf("Number of manifolds: %d\n", numManifolds);

    for (int i = 0; i < numManifolds; i++){
        
        btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();
	
		int numContacts = contactManifold->getNumContacts();

        printf("Manifold %d, %d contacts: \n", i, numContacts);

		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();
            btScalar dist = pt.getDistance();

			printf("Contact %d: position on body A: %f, %f, %f; distance: %f \n", j, ptA.x(),ptA.y(), ptA.z(), dist);
		} 
    }

    //directly query the dispatcher for two objects. The objects don't need to be inserted in to collision world.

}
