#include "BulletDynamics/Featherstone/btMultiBody.h"

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

class  Robot
{   
public:

    btMultiBody* mb = 0;

    btTransform base_transform;
    
    btAlignedObjectArray<btTransform> link_transforms;

    void importURDF(const char* fileName, OpenGLGuiHelper* m_guihelper, double scaling);
    void importSDF(const char* fileName, OpenGLGuiHelper* m_guihelper, double scaling);

    Robot();

    Robot(const char* fileName, OpenGLGuiHelper* m_guihelper, double scaling);

    // ~ Robot();

    void updateGraphics(SimpleOpenGLApp* m_app);

    void setBaseTransform(btTransform tr);

    void setJointPositions(double p[]);
};

