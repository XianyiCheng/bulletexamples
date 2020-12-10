#include "OpenGLWindow/SimpleOpenGL3App.h"
typedef SimpleOpenGL3App SimpleOpenGLApp;

#include "btBulletCollisionCommon.h"

#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "assert.h"
#include <stdio.h>
#include "b3BulletDefaultFileIO.h"

#include "b3ImportMeshUtility.h"
#include  "GLInstanceGraphicsShape.h"
#include "b3ResourcePath.h"
#include <string>

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btAlignedAllocator.h"

static char* gVideoFileName = 0;
static char* gPngFileName = 0;

static b3WheelCallback sOldWheelCB = 0;
static b3ResizeCallback sOldResizeCB = 0;
static b3MouseMoveCallback sOldMouseMoveCB = 0;
static b3MouseButtonCallback sOldMouseButtonCB = 0;
static b3KeyboardCallback sOldKeyboardCB = 0;
//static b3RenderCallback sOldRenderCB = 0;

static float gWidth = 1024;
static float gHeight = 768;

void MyWheelCallback2(float deltax, float deltay)
{
	if (sOldWheelCB)
		sOldWheelCB(deltax, deltay);
}
void MyResizeCallback2(float width, float height)
{
	gWidth = width;
	gHeight = height;

	if (sOldResizeCB)
		sOldResizeCB(width, height);
}
void MyMouseMoveCallback2(float x, float y)
{
	//printf("Mouse Move: %f, %f\n", x, y);

	if (sOldMouseMoveCB)
		sOldMouseMoveCB(x, y);
}
void MyMouseButtonCallback2(int button, int state, float x, float y)
{
	if (sOldMouseButtonCB)
		sOldMouseButtonCB(button, state, x, y);
}

static void MyKeyboardCallback2(int keycode, int state)
{
	//keycodes are in examples/CommonInterfaces/CommonWindowInterface.h
	//for example B3G_ESCAPE for escape key
	//state == 1 for pressed, state == 0 for released.
	// use app->m_window->isModifiedPressed(...) to check for shift, escape and alt keys
	
    //printf("MyKeyboardCallback received key:%c in state %d\n", keycode, state);
	if (sOldKeyboardCB)
		sOldKeyboardCB(keycode, state);
}

int main(int argc, char* argv[])
{
	{
		// OpenGL initialization

		SimpleOpenGLApp* app = new SimpleOpenGLApp("SimpleOpenGL3App", 1024, 768);

		app->m_renderer->getActiveCamera()->setCameraDistance(13);
		app->m_renderer->getActiveCamera()->setCameraPitch(0);
		app->m_renderer->getActiveCamera()->setCameraTargetPosition(0, 0, 0);
		sOldKeyboardCB = app->m_window->getKeyboardCallback();
		app->m_window->setKeyboardCallback(MyKeyboardCallback2);
		sOldMouseMoveCB = app->m_window->getMouseMoveCallback();
		app->m_window->setMouseMoveCallback(MyMouseMoveCallback2);
		sOldMouseButtonCB = app->m_window->getMouseButtonCallback();
		app->m_window->setMouseButtonCallback(MyMouseButtonCallback2);
		sOldWheelCB = app->m_window->getWheelCallback();
		app->m_window->setWheelCallback(MyWheelCallback2);
		sOldResizeCB = app->m_window->getResizeCallback();
		app->m_window->setResizeCallback(MyResizeCallback2);

        // collision world intialization
        btAlignedObjectArray<btCollisionObject> colliders;
        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
        btVector3	worldAabbMin(-1000,-1000,-1000);
        btVector3	worldAabbMax(1000,1000,1000);

        btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);

        btCollisionWorld* collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);

        // create shape

		// box shape 1

		{
            float box_length = 3.0;

            btVector3 pos(0, 0, 0);
			btQuaternion orn(0, 0, 0, 1);
			btVector4 color(0.3, 0.3, 0.3, 0.8);
			btVector3 scaling(1, 1, 1);

            // create box shape
            btBoxShape* boxshape = new btBoxShape(btVector3(box_length,box_length,box_length));
            boxshape->setMargin(0.f);

            // create box collision object
            btCollisionObject* box_collision_obj = new btCollisionObject;
            box_collision_obj->setCollisionShape(boxshape);
            btTransform tr;
            tr.setOrigin(pos);
            tr.setRotation(orn);
            box_collision_obj->setWorldTransform(tr);

            colliders.push_back(*box_collision_obj);
            
            // register box in app
			int boxId = app->registerCubeShape(box_length, box_length, box_length);
			app->m_renderer->registerGraphicsInstance(boxId, pos, orn, color, scaling);
		}

        // box shape 2

		{
            float box_length = 1.0;

            btVector3 pos(0, 3+0.90, 0);
			btQuaternion orn(0, 0, 0, 1);
			btVector4 color(0.7, 0.3, 0.3, 1);
			btVector3 scaling(1, 1, 1);

            // create box shape
            btBoxShape* boxshape = new btBoxShape(btVector3(box_length,box_length,box_length));
            boxshape->setMargin(0.f);

            // create box collision object
            btCollisionObject* box_collision_obj = new btCollisionObject;
            box_collision_obj->setCollisionShape(boxshape);
            btTransform tr;
            tr.setOrigin(pos);
            tr.setRotation(orn);
            box_collision_obj->setWorldTransform(tr);

            colliders.push_back(*box_collision_obj);
            
            // register box in app
			int boxId = app->registerCubeShape(box_length, box_length, box_length);
			app->m_renderer->registerGraphicsInstance(boxId, pos, orn, color, scaling);
		}

        for (int i = 0; i < colliders.size(); i++){
            collisionWorld->addCollisionObject(&colliders[i]);
        }

		app->m_renderer->writeTransforms();
		
		//render scene
		do
		{
			static int frameCount = 0;
			frameCount++;
			
			app->m_renderer->init();
			int upAxis = 1;
			app->m_renderer->updateCamera(upAxis);

			app->m_renderer->renderScene();

            {
                float x = 0;
                float y = 3+0.6+ 0.002*float((frameCount)%300);
                float z = 0;
                float pos[3] = {x, y, z};
			    float orn[4] = {0, 0, 0, 1};
                app->m_renderer->writeSingleInstanceTransformToCPU(pos, orn, 1);
                

                btTransform tr;
                tr.setOrigin(btVector3(x,y,z));
                tr.setRotation(btQuaternion(0, 0, 0, 1));
                colliders[1].setWorldTransform(tr);
            }

            app->m_renderer->writeTransforms();

            // all contacts in the world
            collisionWorld->performDiscreteCollisionDetection();

            int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();


            for (int i = 0; i < numManifolds; i++){
                
                btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
                const btCollisionObject* obA = contactManifold->getBody0();
                const btCollisionObject* obB = contactManifold->getBody1();
            
                int numContacts = contactManifold->getNumContacts();

                for (int j=0;j<numContacts;j++)
                {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);
                    
                    btVector3 ptA = pt.getPositionWorldOnA();
                    btVector3 ptB = pt.getPositionWorldOnB();
                    btScalar dist = pt.getDistance();

                    btVector3 color(1,1,0);
                    btScalar lineWidth = 3;
                    if (dist < 0){
                        color.setValue(1,0,0);
                    }
                    app->m_renderer->drawLine(ptA, ptB, color, lineWidth);
                } 
            }

			app->drawGrid();
			
			app->swapBuffer();

		} while (!app->m_window->requestedExit());

		delete app;

	}
	return 0;
}
