#ifndef BULLET_N_GRAPHICS_H
#define BULLET_N_GRAPHICS_H
    #include "OpenGLWindow/SimpleOpenGL3App.h"
    typedef SimpleOpenGL3App SimpleOpenGLApp;
    #include "OpenGLGuiHelper.h"

    #include "btBulletCollisionCommon.h"

    #include "Bullet3Common/b3Quaternion.h"
    #include "Bullet3Common/b3CommandLineArgs.h"

    #include "b3BulletDefaultFileIO.h"

    #include "b3ImportMeshUtility.h"
    #include  "GLInstanceGraphicsShape.h"
    #include "b3ResourcePath.h"
    #include <string>

    #include "LinearMath/btVector3.h"
    #include "LinearMath/btAlignedObjectArray.h"
    #include "LinearMath/btAlignedAllocator.h"
    
    #include "assert.h"
    #include <stdio.h>
    #include "math.h"
    #include <vector>
#endif

#include "BulletDynamics/Featherstone/btMultiBody.h"



class World {

public:
    
    btAlignedObjectArray<btCollisionObject> colliders;
    btCollisionWorld* m_collisionWorld = 0;

    SimpleOpenGLApp* m_app = 0;
    OpenGLGuiHelper* m_guihelper = 0;

    btMultiBody* mb = 0;
    int object_id = 0;

    World();
    //~World();

    void addCollisionObject(btCollisionObject* colObj);

    void initializeGraphics();
    void renderScene();
    void getContactPoints(std::vector<btManifoldPoint>* pts, int userId2);
    void drawContact(const btManifoldPoint& pt);
    void drawAllContacts();
    void drawContactsOnObject(int userId2);
    void updateObjectTranformation(const btTransform& tr);
    void updateCollisionObjectGraphics();
    void updateAllColliderGraphics();
    void updateCollisionObjects();
    btCollisionShape* ConvexShapefromMesh(std::string file);
    void importURDF(const char* fileName);
    void UpdateURDFTransformations();
};