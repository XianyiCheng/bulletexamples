#include "world.h"
#include "URDF2btMultiBody.h"
#include "robot.h"
#include <iostream>

int main() {

    World w;

    w.initializeGraphics();

    // object box

    {
        float box_length = 1.0;

        btVector3 pos(0, 0.99, 0);
        btQuaternion orn(0, 0, 0, 1);
        btVector4 color(0.7, 0.3, 0.3, 0.9);
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

        // use positive userindex2 to identify object
        box_collision_obj->setUserIndex2(1);

        // w.colliders.push_back(*box_collision_obj);

        w.addCollisionObject(box_collision_obj);
    }

    
    		
    // env box
    {
        btVector3 pos(0, -5, 0);
        btQuaternion orn(0, 0, 0, 1);
        btVector4 color(0.3, 0.3, 0.3, 0.8);
        btVector3 scaling(1, 1, 1);

        // create box shape
        btBoxShape* boxshape = new btBoxShape(btVector3(50,5,50));
        boxshape->setMargin(0.f);

        // create box collision object
        btCollisionObject* box_collision_obj = new btCollisionObject;
        box_collision_obj->setCollisionShape(boxshape);
        btTransform tr;
        tr.setOrigin(pos);
        tr.setRotation(orn);
        box_collision_obj->setWorldTransform(tr);

        // w.addCollisionObject(box_collision_obj);
    }

    // w.importURDF("/home/xianyi/libraries/bullet3/data/kuka_iiwa/model.urdf");

    w.updateCollisionObjects();
    
    w.updateCollisionObjectGraphics();

    // Robot kuka_robot("/home/xianyi/libraries/bullet3/data/kuka_iiwa/model.urdf", w.m_guihelper, 10);
    Robot kuka_robot("/home/xianyi/projects/ddhand_abb_120/ddhand_irb120_3_58.urdf", w.m_guihelper, 10);

    kuka_robot.setBaseTransform(btTransform(btQuaternion(0.7071, 0, 0, -0.7071), btVector3(0, 2, 0)));

    std::cout << "Num link: " << kuka_robot.mb->getNumDofs() << std::endl;

    double p[33];
    for (int k=0;k<33;k++){
        p[k]=0;
    }

    // try render
    do
    {
        static int frameCount = 0;
        frameCount++;

        w.m_app->m_renderer->init();
        int upAxis = 1;
        w.m_app->m_renderer->updateCamera(upAxis);

        w.m_app->m_renderer->renderScene();

        // box 2

        // {
        //     float y = 0.6+ 0.002*float((frameCount)%300);
        //     btTransform tr(btQuaternion(0, 0, 0, 1), btVector3(0,y,0));
        //     w.updateObjectTranformation(tr);
        // }

        {
            // for (int i = 0; i < kuka_robot.mb->getNumLinks()+1; i++){
            //     p[i] = 0.002*float((frameCount)%600);
            // }

            kuka_robot.setJointPositions(p);
            kuka_robot.updateGraphics(w.m_app);
        }


        w.updateAllColliderGraphics();

        // draw all contacts
        w.drawAllContacts();

        // w.m_app->drawGrid();
        
        w.m_app->swapBuffer();


    } while (!w.m_app->m_window->requestedExit());
    delete w.m_app;
}
