#include "world.h"
#include "URDF2btMultiBody.h"
#include "robot.h"
#include <iostream>

int main() {

    World w;

    w.initializeGraphics();

    // Robot robot("/home/xianyi/libraries/bullet3/data/kuka_iiwa/kuka_with_gripper2.sdf", w.m_guihelper, 10);
    Robot robot("/home/xianyi/projects/ddhand_abb_120/ddhand_irb120_3_58.urdf", w.m_guihelper, 10);
    // Robot robot("/home/xianyi/projects/ddhand_abb_120/ddhand_irb120.sdf", w.m_guihelper, 10);

    robot.setBaseTransform(btTransform(btQuaternion(0.7071, 0, 0, -0.7071), btVector3(0, 2, 0)));

    std::cout << "Num link: " << robot.mb->getNumLinks() << std::endl;

    for (int i = 0; i < robot.mb->getNumLinks(); i++){
        
        // std::string linkname(robot.mb->getLink(i).m_linkName);
        std::cout <<"Link " << i << ": joint type " << robot.mb->getLink(i).m_jointType << std::endl;
        // std::cout <<"Link " << i << ": joint type " << robot.mb->getLink(i).m_jointName << std::endl;
        // joint type: eRevolute = 0, ePrismatic = 1, eSpherical = 2, ePlanar = 3, eFixed = 4, eInvalid
    }

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
            // for (int i = 0; i < robot.mb->getNumLinks(); i++){
            // for (int i = 0; i < 6; i++){
            //     p[i] = 0.002*float((frameCount)%1000);
            // }

            // p[13] = 0.5-0.002*float((frameCount)%1000);
            // p[20] = 0.5-0.002*float((frameCount)%1000);
            // p[21] = 0.5-0.002*float((frameCount)%1000);

            p[14] = 0.5-0.002*float((frameCount)%1000);
            p[15] = 0.5-0.002*float((frameCount)%1000);
            p[18] = 0.5-0.002*float((frameCount)%1000);
            p[19] = 0.5-0.002*float((frameCount)%1000);

            robot.setJointPositions(p);
            robot.updateGraphics(w.m_app);
        }


        w.updateAllColliderGraphics();

        // draw all contacts
        w.drawAllContacts();

        // w.m_app->drawGrid();
        
        w.m_app->swapBuffer();


    } while (!w.m_app->m_window->requestedExit());
    delete w.m_app;
}
