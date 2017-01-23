// Physics.

#ifndef _TFPS_PHYSICS_H
#define _TFPS_PHYSICS_H

#include "compatibility.h"
#include <btBulletDynamicsCommon.h>
//#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <list>
#include <map>
#include <set>

#define COLLIDE_WITH_EVERYONE ((short)0x7fff)

class Box;
class Sphere;
class PhysicsObject;
class PhysicsWorld;

class PhysicsObject {
public:
	PhysicsWorld* parent;
	btCollisionShape* shape;
	btDefaultMotionState* motionState;
	btRigidBody* rigidBody;
//	btGhostObject* ghostObject;
//	btCollisionObject* collisionObject;

	void getPos(Real* xyz);
	void setPos(Real x, Real y, Real z);
	void getAxisAngle(Real* axis_angle);
	void setAxisAngle(Real x, Real y, Real z, Real t);
	void setPosAxisAngle(Real xx, Real yy, Real zz, Real x, Real y, Real z, Real t);
	void getLinearVelocity(Real* xyz);
	void setLinearVelocity(Real* xyz);
	void convertIntoReferenceFrame();
	void applyForce(Real x, Real y, Real z);
	void applyCentralImpulse(Real x, Real y, Real z);
	void setAngularFactor(Real x, Real y, Real z);
	void setGravity(Real x, Real y, Real z);
	void setKinematic(bool is_kinematic);
	void removeFromWorld();
};

class Box : public PhysicsObject {
public:
	Box(PhysicsWorld* parent, Real* bounds, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
};

class Sphere : public PhysicsObject {
public:
	Sphere(PhysicsWorld* parent, Real radius, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
};

class ConvexHull : public PhysicsObject {
public:
	ConvexHull(PhysicsWorld* parent, Real* position, Real* rotation_quaternion, int point_count, Real* points, Real mass, int collision_group);
};

class Capsule : public PhysicsObject {
public:
	Capsule(PhysicsWorld* parent, Real radius, Real height, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
};

class PlayerShape : public PhysicsObject {
public:
	PlayerShape(PhysicsWorld* parent, Real radius, Real cone_height, Real torso_height, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
};

class BvhTriangleMesh : public PhysicsObject {
	btTriangleMesh* mesh;

public:
	BvhTriangleMesh(PhysicsWorld* parent, Real* position, Real* rotation_quaternion, int triangle_count, Real* triangles, int collision_group);
};

class PhysicsWorld {
public:
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	std::list<PhysicsObject*> objects;

	PhysicsWorld();
	void step(Real dt);
	bool rayCast(btVector3 start, btVector3 end, btVector3& result, const btCollisionObject*&, int collision_mask);
	bool checkForContact(PhysicsObject* a, PhysicsObject* b, int collision_mask);
	std::set<std::pair<PhysicsObject*, PhysicsObject*>>* listAllCollidingPairs();
};

// Now for the C API, for use with ctypes and Python.

extern "C" EXPORT PhysicsWorld* PhysicsWorld_new(void);
extern "C" EXPORT void PhysicsWorld_step(PhysicsWorld* w, Real dt);
extern "C" EXPORT int PhysicsWorld_rayCast(PhysicsWorld* w, Real* start, Real* end, Real* hit, PhysicsObject** hit_object, int collision_mask);
extern "C" EXPORT int PhysicsWorld_checkForContact(PhysicsWorld* w, PhysicsObject* a, PhysicsObject* b, int collision_mask);
extern "C" EXPORT int PhysicsWorld_listAllCollidingPairs(PhysicsWorld* w, PhysicsObject*** pairs);
extern "C" EXPORT void delete_colliding_pairs_list(PhysicsObject** pairs);

extern "C" EXPORT Box* Box_new(PhysicsWorld* parent, Real* bounds, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
extern "C" EXPORT Sphere* Sphere_new(PhysicsWorld* parent, Real radius, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
extern "C" EXPORT ConvexHull* ConvexHull_new(PhysicsWorld* parent, Real* position, Real* rotation_quaternion, int point_count, Real* points, Real mass, int collision_group);
extern "C" EXPORT Capsule* Capsule_new(PhysicsWorld* parent, Real radius, Real height, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
extern "C" EXPORT PlayerShape* PlayerShape_new(PhysicsWorld* parent, Real radius, Real cone_height, Real torso_height, Real* position, Real* rotation_quaternion, Real mass, int collision_group);
extern "C" EXPORT BvhTriangleMesh* BvhTriangleMesh_new(PhysicsWorld* parent, int triangle_count, Real* triangles, Real* position, Real* rotation_quaternion, int collision_group);
extern "C" EXPORT void PhysicsObject_removeFromWorld(PhysicsObject* obj);
extern "C" EXPORT void PhysicsObject_getPos(PhysicsObject* obj, Real* xyz);
extern "C" EXPORT void PhysicsObject_setPos(PhysicsObject* obj, Real x, Real y, Real z);
extern "C" EXPORT void PhysicsObject_getAxisAngle(PhysicsObject* obj, Real* axis_angle);
extern "C" EXPORT void PhysicsObject_setAxisAngle(PhysicsObject* obj, Real x, Real y, Real z, Real t);
extern "C" EXPORT void PhysicsObject_setPosAxisAngle(PhysicsObject* obj, Real xx, Real yy, Real zz, Real x, Real y, Real z, Real t);
extern "C" EXPORT void PhysicsObject_getLinearVelocity(PhysicsObject* obj, Real* xyz);
extern "C" EXPORT void PhysicsObject_setLinearVelocity(PhysicsObject* obj, Real vx, Real vy, Real vz);
extern "C" EXPORT void PhysicsObject_convertIntoReferenceFrame(PhysicsObject* obj);
extern "C" EXPORT void PhysicsObject_applyForce(PhysicsObject* obj, Real x, Real y, Real z);
extern "C" EXPORT void PhysicsObject_applyCentralImpulse(PhysicsObject* obj, Real x, Real y, Real z);
extern "C" EXPORT void PhysicsObject_setAngularFactor(PhysicsObject* obj, Real x, Real y, Real z);
extern "C" EXPORT void PhysicsObject_setGravity(PhysicsObject* obj, Real x, Real y, Real z);
extern "C" EXPORT void PhysicsObject_setKinematic(PhysicsObject* obj, int is_kinematic);

#endif

