#ifndef _URDF2BULLET_H
#define _URDF2BULLET_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include <string>
#include "Importers/ImportURDFDemo/URDFJointTypes.h"  //for UrdfMaterialColor cache

class btVector3;
class btTransform;
class btMultiBodyDynamicsWorld;
class btDiscreteDynamicsWorld;
class btTransform;

class URDFImporterInterface;
class MultiBodyCreationInterface;



struct UrdfVisualShapeCache
{
	btAlignedObjectArray<UrdfMaterialColor> m_cachedUrdfLinkColors;
	btAlignedObjectArray<int> m_cachedUrdfLinkVisualShapeIndices;
};

void URDF2BulletMultiBody(const URDFImporterInterface& u2b,
						MultiBodyCreationInterface& creationCallback,
						const btTransform& rootTransformInWorldSpace,
						bool createMultiBody,
						const char* pathPrefix,
						int flags = 0,
						UrdfVisualShapeCache* cachedLinkGraphicsShapes = 0);

#endif  //_URDF2BULLET_H

void forwardKinematics_worldtransforms(btMultiBody* mb, btAlignedObjectArray<btTransform>& worldtransforms);

void forwardKinematics_worldtransforms(btMultiBody* mb, btTransform& base_transform, btAlignedObjectArray<btTransform>& link_transforms);