#include "world.h"
#include "URDF2btMultiBody.h"

#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "Importers/ImportURDFDemo/BulletUrdfImporter.h"
// #include "Importers/ImportURDFDemo/URDF2Bullet.h"
#include "Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "Importers/ImportURDFDemo/ImportURDFSetup.h"

#include "CommonInterfaces/CommonParameterInterface.h"

#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"

#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"



World::World(){
        
        // collision world intialization
        //btAlignedObjectArray<btCollisionObject> colliders;

        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
        btVector3	worldAabbMin(-1000,-1000,-1000);
        btVector3	worldAabbMax(1000,1000,1000);

        btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);

        this->m_collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
}

void World::initializeGraphics(){

    if (this->m_app == 0){

        this->m_app = new SimpleOpenGLApp("World", 1024, 768);
        this->m_app->setUpAxis(1);
        
        this->m_app->m_renderer->getActiveCamera()->setCameraDistance(13);
        this->m_app->m_renderer->getActiveCamera()->setCameraPitch(0);
        this->m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(0, 0, 0);

        this->m_guihelper = new OpenGLGuiHelper(this->m_app, false);
    }

}

void World::updateCollisionObjectGraphics(){
    for (int i = 0; i < this->colliders.size(); i++){

        //printf("COllision object user index: %d \n", this->colliders[i].getUserIndex());

        // use userindex2 to distinguish object and environment
        btVector4 color(0.4, 0.4, 0.4, 0.8); // default: environment
        if (this->colliders[i].getUserIndex2() > 0){
            color.setValue(0.7, 0.3, 0.3, 0.9); // object
        } else if (this->colliders[i].getUserIndex2() != -1) {
            color.setValue(1, 1, 1, 1);
        }

        if(this->colliders[i].getCollisionShape()->getUserIndex()>0){
            // if graphics shape has been registered
            int useridx = m_app->m_renderer->registerGraphicsInstance(this->colliders[i].getCollisionShape()->getUserIndex(), 
                this->colliders[i].getWorldTransform().getOrigin(), this->colliders[i].getWorldTransform().getRotation(), color, btVector3(1, 1, 1));
            this->colliders[i].setUserIndex(useridx);
        } else {
            this->m_guihelper->createCollisionShapeGraphicsObject(this->colliders[i].getCollisionShape());
            // m_app->m_renderer->registerGraphicsInstance(this->colliders[i].getCollisionShape()->getUserIndex(), 
            //  this->colliders[i].getWorldTransform().getOrigin(), this->colliders[i].getWorldTransform().getRotation(), color, btVector3(1, 1, 1));
            this->m_guihelper->createCollisionObjectGraphicsObject(&(this->colliders[i]), color);
        }
        

        //printf("COllision object user index: %d \n", this->colliders[i].getUserIndex());
    }
}

void World::getContactPoints(std::vector<btManifoldPoint>* pts, int userId2 = -1){

    this->m_collisionWorld->performDiscreteCollisionDetection();

    int numManifolds = this->m_collisionWorld->getDispatcher()->getNumManifolds();


    for (int i = 0; i < numManifolds; i++){
        
        btPersistentManifold* contactManifold = this->m_collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        if (userId2!=-1){
            // std::cout<< obA->getUserIndex2() << obB->getUserIndex2() << std::endl;
            if (obA->getUserIndex2()!=userId2 && obB->getUserIndex2()!=userId2){
                continue;
            }
        }
    
        int numContacts = contactManifold->getNumContacts();

        for (int j=0;j<numContacts;j++)
        {

            btManifoldPoint pt = contactManifold->getContactPoint(j);
            if (userId2!=-1){
                if (obB->getUserIndex2()==userId2){
                    pt.m_normalWorldOnB *= -1;
                }
            }
            pts->push_back(pt);
            
        } 
    }
}

void World::drawContact(const btManifoldPoint& pt){
    btVector3 ptA = pt.getPositionWorldOnA();
    btVector3 ptB = pt.getPositionWorldOnB();
    btScalar dist = pt.getDistance();

    btVector3 color(1,1,0);
    btScalar lineWidth = 3;
    if (dist < 0){
        color.setValue(1,0,0);
    }
    this->m_app->m_renderer->drawLine(ptA, ptB, color, lineWidth);

    // draw normal
    btVector3 nB = 0.3*pt.m_normalWorldOnB;
    btVector3 ptB2 = ptB + nB;
    color.setValue(0,1,0);
    this->m_app->m_renderer->drawLine(ptB, ptB2, color, lineWidth);

}

void World::drawAllContacts(){

    std::vector<btManifoldPoint> pts;
    this->getContactPoints(&pts);
  
    int numContacts = pts.size();

    for (int j=0;j<numContacts;j++)
    {
        this->drawContact(pts[j]);
        // std::cout << "contact distance " << pts[j].getDistance() << std::endl;
    } 
}

void World::drawContactsOnObject(int userId2){

    std::vector<btManifoldPoint> pts;
    this->getContactPoints(&pts, userId2);
  
    int numContacts = pts.size();

    for (int j=0;j<numContacts;j++)
    {
        this->drawContact(pts[j]);
    } 
}

void World::addCollisionObject(btCollisionObject* colObj){

    int index = this->colliders.size();
    this->colliders.push_back(*colObj);
    //this->m_collisionWorld->addCollisionObject(&(this->colliders[index]));
    //return index;
}

void World::updateCollisionObjects(){
    
    for (int i = 0; i < this->colliders.size(); i++){
        this->m_collisionWorld->addCollisionObject(&(this->colliders[i]));
    }
}

void World::updateObjectTranformation(const btTransform& tr){
    
    colliders[this->object_id].setWorldTransform(tr);

    if (this->m_app != 0){

        this->m_app->m_renderer->writeSingleInstanceTransformToCPU(tr.getOrigin(), tr.getRotation(), colliders[this->object_id].getUserIndex());
        this->m_app->m_renderer->writeTransforms();
    }
        
}

void World::updateAllColliderGraphics(){
    
    if (this->m_app != 0){

        for (int i = 0; i < this->colliders.size(); i++){

            btTransform tr = this->colliders[i].getWorldTransform();

            this->m_app->m_renderer->writeSingleInstanceTransformToCPU(tr.getOrigin(), tr.getRotation(), this->colliders[i].getUserIndex());
            
        }
        this->m_app->m_renderer->writeTransforms();
    }
        
}

btCollisionShape* World::ConvexShapefromMesh(std::string file){
    
    // mesh shape
    if (this->m_app == 0){
        this->initializeGraphics();
    }
    
    GLInstanceGraphicsShape* glmesh = new GLInstanceGraphicsShape;

    // const char* f = "/home/xianyi/libraries/bullet3/data/torus/torus.obj";

    float position[4] = {0, 5.5, 0, 1};
    //float orn[4] = {0, 0, 0, 1};
    float a = 3.14/4;
    float orn[4] = {sin(a/2), 0, 0, cos(a/2)};
    float color[4] = {0.3, 0.3, 0.8, 0.9};
    float scaling[4] = {1, 1, 1, 1};

    b3ImportMeshData meshData;
    b3BulletDefaultFileIO fileIO;
    if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(file, meshData, &fileIO))
    {

        glmesh = meshData.m_gfxShape;

        //apply the geometry scaling
        for (int i = 0; i < glmesh->m_vertices->size(); i++)
        {
            glmesh->m_vertices->at(i).xyzw[0] *= scaling[0];
            glmesh->m_vertices->at(i).xyzw[1] *= scaling[1];
            glmesh->m_vertices->at(i).xyzw[2] *= scaling[2];
        }
    }

    int textureIndex = -1;
    int shapeId = -1;

    btCollisionShape* collisionShape = 0;

    btAlignedObjectArray<btVector3> convertedVerts;
    convertedVerts.reserve(glmesh->m_numvertices);
    for (int i = 0; i < glmesh->m_numvertices; i++)
    {
        convertedVerts.push_back(btVector3(
            glmesh->m_vertices->at(i).xyzw[0],
            glmesh->m_vertices->at(i).xyzw[1],
            glmesh->m_vertices->at(i).xyzw[2]));
    }

    // force convex shape
    {
    
        btConvexHullShape* convexHull = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
        
        convexHull->optimizeConvexHull();
        //convexHull->initializePolyhedralFeatures();
        //convexHull->setMargin(m_data->m_globalDefaults.m_defaultCollisionMargin);
        collisionShape = convexHull;
        
        shapeId = this->m_app->m_renderer->registerShape(&(glmesh->m_vertices->at(0).xyzw[0]),
                                                glmesh->m_numvertices,
                                                &(glmesh->m_indices->at(0)),
                                                glmesh->m_numIndices,
                                                1,
                                                textureIndex);
        
    }
    
    // int srcidx = this->m_app->m_renderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
    collisionShape->setUserIndex(shapeId);


    return collisionShape;
}
