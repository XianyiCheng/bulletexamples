#include "robot.h"
#include "URDF2btMultiBody.h"

#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "Importers/ImportURDFDemo/BulletUrdfImporter.h"
// #include "BulletUrdfImporter.h"
#include "Importers/ImportURDFDemo/URDF2Bullet.h"
#include "Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "Importers/ImportURDFDemo/ImportURDFSetup.h"

#include "CommonInterfaces/CommonParameterInterface.h"

#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"

#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

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
#include <iostream>

void Robot::importURDF(const char* fileName, OpenGLGuiHelper* m_guihelper, double scaling){
    
    int flags = 0;

    // graphics object already added to the gui, and register with collision object with userIndex
	BulletURDFImporter u2b(m_guihelper, 0, 0, scaling, flags);

	bool loadOk = u2b.loadURDF(fileName);
    
    // u2b.CheckLinks();

	if (loadOk)
	{
		btTransform identityTrans;
		identityTrans.setIdentity();

		{
			MyMultiBodyCreator creation(m_guihelper);
            // u2b.CheckVisualShapes();
			URDF2BulletMultiBody(u2b, creation, identityTrans, 1, u2b.getPathPrefix());
            // u2b.CheckLinks();
			this->mb = creation.getBulletMultiBody();
		}

        for (int i = 0; i < this->mb->getNumLinks(); i++){
            
            std::cout << "Link " << i << ": " << u2b.getLinkName(i) << std::endl;
        }

    }

}

void Robot::importSDF(const char* fileName, OpenGLGuiHelper* m_guihelper, double scaling){
    
    int flags = 0;

    // graphics object already added to the gui, and register with collision object with userIndex
	BulletURDFImporter u2b(m_guihelper, 0, 0, scaling, flags);

	bool loadOk = u2b.loadSDF(fileName);
    
    // u2b.CheckLinks();

	if (loadOk)
	{
		btTransform identityTrans;
		identityTrans.setIdentity();

		btTransform rootTrans;
		rootTrans.setIdentity();
        int NumModels = u2b.getNumModels();
		for (int m = 0; m < NumModels; m++)
		{
			u2b.activateModel(m);

			btMultiBody* mb = 0;

			MyMultiBodyCreator creation(m_guihelper);

			u2b.getRootTransformInWorld(rootTrans);
			URDF2BulletMultiBody(u2b, creation, rootTrans, 1, u2b.getPathPrefix(), CUF_USE_SDF);
			mb = creation.getBulletMultiBody();

            for (int i = 0; i < mb->getNumLinks(); i++){
                
                std::cout << "Link " << i << "name " << u2b.getLinkName(i) << std::endl;
            }

			if (mb)
			{
				std::string* name = new std::string(u2b.getLinkName(u2b.getRootLinkIndex()));
				mb->setBaseName(name->c_str());
				//create motors for each btMultiBody joint
				int numLinks = mb->getNumLinks();
                std::cout << name->c_str() << ", NumLinks: " << numLinks << std::endl;
            }

            this->mb = mb;
        }
    }

}

Robot::Robot(const char* fileName, OpenGLGuiHelper* m_guihelper, double scaling){
    std::string filestr(fileName);
    if (filestr.compare(filestr.length()-3,3,"sdf") == 0){
        this->importSDF(fileName, m_guihelper, scaling);
    } else {
        this->importURDF(fileName, m_guihelper, scaling);
    }
    
}

void Robot::updateGraphics(SimpleOpenGLApp* m_app){
    
    if (m_app != 0){

        // this->mb->getBaseCollider()->setWorldTransform(this->base_transform);

        // for (int i = 0; i < this->mb->getNumLinks(); i++){
        //     this->mb->getLinkCollider(i)->setWorldTransform(this->link_transforms[i]);
        // }

        {
            btTransform tr = this->base_transform;
            m_app->m_renderer->writeSingleInstanceTransformToCPU(tr.getOrigin(), tr.getRotation(), this->mb->getBaseCollider()->getUserIndex());
        }

        for (int i = 0; i < this->mb->getNumLinks(); i++){
            btTransform tr = this->link_transforms[i];
            btTransform ttr;
            // bool isvisualinstance = m_app->m_renderer->readSingleInstanceTransformToCPU(ttr.getOrigin(), ttr.getRotation(), this->mb->getLinkCollider(i)->getUserIndex());
            // std::cout << "Link " << i << ": " << isvisualinstance << std::endl;

            m_app->m_renderer->writeSingleInstanceTransformToCPU(tr.getOrigin(), tr.getRotation(), this->mb->getLinkCollider(i)->getUserIndex());

        }

        m_app->m_renderer->writeTransforms();
    
    }
}

void Robot::setBaseTransform(btTransform tr){
    this->mb->setBaseWorldTransform(tr);
    forwardKinematics_worldtransforms(this->mb, this->base_transform, this->link_transforms);
}

void Robot::setJointPositions(double p[]){

    for (int i = 0; i < this->mb->getNumLinks(); i++){
        this->mb->setJointPos(i, p[i]);
    }

    forwardKinematics_worldtransforms(this->mb, this->base_transform, this->link_transforms);
}
