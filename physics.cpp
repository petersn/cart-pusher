// Physics.

using namespace std;
#include <iostream>
#include <vector>

#include "physics.h"
#include "graphics.h"
#include <math.h>

PhysicsWorld::PhysicsWorld() {
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
//	broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	dynamicsWorld->setGravity(btVector3(0, 0, -9.8));
}

void PhysicsWorld::step(Real dt) {
	dynamicsWorld->stepSimulation(dt, 60);
}

bool PhysicsWorld::rayCast(const btVector3& start, const btVector3& end, btVector3& result, btVector3& normal, const btCollisionObject*& hit_collision_object, int collision_mask) {
	btCollisionWorld::ClosestRayResultCallback res(start, end);
	res.m_collisionFilterGroup = collision_mask;
	res.m_collisionFilterMask = collision_mask;
	dynamicsWorld->rayTest(start, end, res);
	if (res.hasHit()) {
		result = res.m_hitPointWorld;
		// Extract the collision surface normal via m_hitNormalWorld.
		normal = res.m_hitNormalWorld;
		hit_collision_object = res.m_collisionObject;
		return true;
	}
	return false;
}

bool PhysicsWorld::checkForContact(PhysicsObject* a, PhysicsObject* b, int collision_mask) {
//	btCollisionWorld::ContactResultCallback res;
//	res.m_collisionFilterGroup = collision_mask;
//	res.m_collisionFilterMask = collision_mask;
//	dynamicsWorld->contactPairTest(a->rigidBody, b->rigidBody, res);
//	assert(false); // Doesn't work yet. XXX
//	if (res.hasHit()) {
//		// TODO: Read out other properties of res, like normals here.
//		return true;
//	}
	return false;
}

bool PhysicsWorld::convexSweepTest(Real radius, btVector3 start, btVector3 end, btVector3& result, btVector3& normal, const btCollisionObject*& hit_collision_object, int collision_mask) {
	btCollisionWorld::ClosestConvexResultCallback res(start, end);
	res.m_collisionFilterGroup = collision_mask;
	res.m_collisionFilterMask = collision_mask;
	btSphereShape shape(radius);
	btTransform trans_from, trans_to;
	trans_from.setOrigin(start);
	trans_to.setOrigin(end);
	dynamicsWorld->convexSweepTest(&shape, trans_from, trans_to, res);
	if (res.hasHit()) {
		result = res.m_hitPointWorld;
		normal = res.m_hitNormalWorld;
		hit_collision_object = res.m_hitCollisionObject;
		return true;
	}
	return false;
}

set<pair<PhysicsObject*, PhysicsObject*>>* PhysicsWorld::listAllCollidingPairs() {
	// Produce a mapping of rigidBodys to containing PhysicsObject.
	// PERF: Consider caching this map rather than rebuilding it all the time.
	map<btCollisionObject*, PhysicsObject*> object_mapping;
	for (auto obj : objects)
		object_mapping[obj->rigidBody] = obj;
	// This is the data structure containing pairs of colliding objects we're going to return.
	auto colliding_pairs = new set<pair<PhysicsObject*, PhysicsObject*>>();
	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0.f) {
				colliding_pairs->insert(pair<PhysicsObject*, PhysicsObject*>(object_mapping[obA], object_mapping[obB]));
//				const btVector3& ptA = pt.getPositionWorldOnA();
//				const btVector3& ptB = pt.getPositionWorldOnB();
//				const btVector3& normalOnB = pt.m_normalWorldOnB;
			}
		}
	}
	return colliding_pairs;
}

void PhysicsObject::removeFromWorld() {
	if (rigidBody != nullptr)
		parent->dynamicsWorld->removeRigidBody(rigidBody);
//	if (ghostObject != nullptr)
//		parent->dynamicsWorld->removeGhostObject(ghostObject);
	delete shape;
	delete motionState;
	delete rigidBody;
//	delete ghostObject;
	// This is apparently kosher.
	// BUT: These instances must all allocated with new, and not new[], and never as a local, or otherhow.
	delete this;
}

void PhysicsObject::getPos(Real* xyz) {
	btTransform trans;
	motionState->getWorldTransform(trans);
	auto origin = trans.getOrigin();
	xyz[0] = origin.getX();
	xyz[1] = origin.getY();
	xyz[2] = origin.getZ();
}

void PhysicsObject::setPos(Real x, Real y, Real z) {
	btTransform trans;
	motionState->getWorldTransform(trans);
//	auto origin = trans.getOrigin();
//	xyz[0] = origin.getX();
//	xyz[1] = origin.getY();
//	xyz[2] = origin.getZ();
	trans.setOrigin(btVector3(x, y, z));
	motionState->setWorldTransform(trans);
	rigidBody->setWorldTransform(trans);
}

void PhysicsObject::getAxisAngle(Real* axis_angle) {
	btTransform trans;
	motionState->getWorldTransform(trans);
	btQuaternion quat = trans.getRotation();
	axis_angle[0] = quat.getX();
	axis_angle[1] = quat.getY();
	axis_angle[2] = quat.getZ();
	axis_angle[3] = quat.getAngle();
}

void PhysicsObject::setAxisAngle(Real x, Real y, Real z, Real t) {
	btTransform trans;
	motionState->getWorldTransform(trans);
//	trans.setOrigin(btVector3(xx, yy, zz));
	trans.setRotation(btQuaternion(btVector3(x, y, z), t));
	motionState->setWorldTransform(trans);
	rigidBody->setWorldTransform(trans);
}

void PhysicsObject::setPosAxisAngle(Real xx, Real yy, Real zz, Real x, Real y, Real z, Real t) {
	btTransform trans;
	motionState->getWorldTransform(trans);
	trans.setOrigin(btVector3(xx, yy, zz));
	trans.setRotation(btQuaternion(btVector3(x, y, z), t));
	motionState->setWorldTransform(trans);
	rigidBody->setWorldTransform(trans);
}

void PhysicsObject::getLinearVelocity(Real* xyz) {
	auto vel = rigidBody->getLinearVelocity();
	xyz[0] = vel.getX();
	xyz[1] = vel.getY();
	xyz[2] = vel.getZ();
}

void PhysicsObject::setLinearVelocity(Real* velocity) {
	rigidBody->setLinearVelocity(btVector3(velocity[0], velocity[1], velocity[2]));
	rigidBody->btCollisionObject::setActivationState(ACTIVE_TAG);
}

void PhysicsObject::getAngularVelocity(Real* velocity) {
	auto vel = rigidBody->getAngularVelocity();
	velocity[0] = vel.getX();
	velocity[1] = vel.getY();
	velocity[2] = vel.getZ();
}

void PhysicsObject::setAngularVelocity(Real* velocity) {
	rigidBody->setAngularVelocity(btVector3(velocity[0], velocity[1], velocity[2]));
	rigidBody->btCollisionObject::setActivationState(ACTIVE_TAG);
}

void PhysicsObject::setLocalScaling(Real* scaling) {
	shape->setLocalScaling(btVector3(scaling[0], scaling[1], scaling[2]));
	rigidBody->btCollisionObject::setActivationState(ACTIVE_TAG);
}

void PhysicsObject::convertIntoReferenceFrame() {
	btTransform trans;
	motionState->getWorldTransform(trans);
	btVector3 origin = trans.getOrigin();
	glTranslatef(origin.getX(), origin.getY(), origin.getZ());
	btQuaternion rotation = trans.getRotation();
	Real w = rotation.getW();
	Real x = rotation.getX();
	Real y = rotation.getY();
	Real z = rotation.getZ();
	glRotatef(360.0 * acosf(w) / M_PI, x, y, z);
}

void PhysicsObject::applyForce(Real x, Real y, Real z) {
	rigidBody->applyCentralForce(btVector3(x, y, z));
	// Call the parent class (btCollisionObject)'s setActivationState.
	// According to the docs we should do this rather than calling the rigidBody's setActivationState, because it also sets some timer that keeps the object awake for a little bit.
	rigidBody->btCollisionObject::setActivationState(ACTIVE_TAG);
	//rigidBody->setLinearVelocity(btVector3(x, y, z));
}

void PhysicsObject::applyCentralImpulse(Real x, Real y, Real z) {
	rigidBody->applyCentralImpulse(btVector3(x, y, z));
	rigidBody->btCollisionObject::setActivationState(ACTIVE_TAG);
}

void PhysicsObject::setLinearFactor(Real x, Real y, Real z) {
	rigidBody->setLinearFactor(btVector3(x, y, z));
}

void PhysicsObject::setAngularFactor(Real x, Real y, Real z) {
	rigidBody->setAngularFactor(btVector3(x, y, z));
}

void PhysicsObject::setGravity(Real x, Real y, Real z) {
	rigidBody->setGravity(btVector3(x, y, z));
}

void PhysicsObject::setKinematic(bool is_kinematic) {
	auto flags = rigidBody->getCollisionFlags();
	if (is_kinematic)
		flags |= btCollisionObject::CF_KINEMATIC_OBJECT;
	else
		flags &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
	rigidBody->setCollisionFlags(flags);
}

Box::Box(PhysicsWorld* parent, Real* sizes, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	this->parent = parent;
	// Here we use the convention that sizes are the total thicknesses, but Bullet uses half-extents.
	shape = new btBoxShape(btVector3(sizes[0]/2.0, sizes[1]/2.0, sizes[2]/2.0));
	btVector3 rotation_axis(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);
	auto rotation = btQuaternion(rotation_axis, rotation_quaternion[3]);
//	auto rotation = btQuaternion(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);//, rotation_quaternion[3]);
	motionState = new btDefaultMotionState(btTransform(rotation, btVector3(position[0], position[1], position[2])));
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
	rigidBody = new btRigidBody(rigidBodyCI);
//	rigidBody->setFriction(2.0);
	// Add ourselves into the parent's simulation.
	parent->dynamicsWorld->addRigidBody(rigidBody, collision_group, COLLIDE_WITH_EVERYONE);
	parent->objects.push_back(this);
}

Sphere::Sphere(PhysicsWorld* parent, Real radius, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	this->parent = parent;
	// Here we use the convention that sizes are the total thicknesses, but Bullet uses half-extents.
	shape = new btSphereShape(radius);
	btVector3 rotation_axis(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);
	auto rotation = btQuaternion(rotation_axis, rotation_quaternion[3]);
	motionState = new btDefaultMotionState(btTransform(rotation, btVector3(position[0], position[1], position[2])));
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
	rigidBody = new btRigidBody(rigidBodyCI);
//	rigidBody->setFriction(2.0);
	// Add ourselves into the parent's simulation.
	parent->dynamicsWorld->addRigidBody(rigidBody, collision_group, COLLIDE_WITH_EVERYONE);
	parent->objects.push_back(this);
}

ConvexHull::ConvexHull(PhysicsWorld* parent, Real* position, Real* rotation_quaternion, int point_count, Real* points, Real mass, int collision_group) {
	this->parent = parent;
	btConvexHullShape* conv_hull = new btConvexHullShape();
	shape = conv_hull;
	for (int i = 0; i < point_count; i++)
		conv_hull->addPoint(btVector3(points[3*i], points[3*i + 1], points[3*i + 2]));
	btVector3 rotation_axis(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);
	auto rotation = btQuaternion(rotation_axis, rotation_quaternion[3]);
	motionState = new btDefaultMotionState(btTransform(rotation, btVector3(position[0], position[1], position[2])));
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
	rigidBody = new btRigidBody(rigidBodyCI);
//	rigidBody->setFriction(2.0);
	// Add ourselves into the parent's simulation.
	parent->dynamicsWorld->addRigidBody(rigidBody, collision_group, COLLIDE_WITH_EVERYONE);
	parent->objects.push_back(this);
}

Capsule::Capsule(PhysicsWorld* parent, Real radius, Real height, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	this->parent = parent;
	// Here we use the convention that sizes are the total thicknesses, but Bullet uses half-extents.
	shape = new btCapsuleShape(radius, height);
	btVector3 rotation_axis(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);
	auto rotation = btQuaternion(rotation_axis, rotation_quaternion[3]);
	motionState = new btDefaultMotionState(btTransform(rotation, btVector3(position[0], position[1], position[2])));
	btVector3 inertia(0, 0, 0);
	// XXX: Warning: I make it so that all capsules have infinite moment of inertia!
//	shape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
	rigidBody = new btRigidBody(rigidBodyCI);
	//	rigidBody->setFriction(2.0);
	// Add ourselves into the parent's simulation.
	parent->dynamicsWorld->addRigidBody(rigidBody, collision_group, COLLIDE_WITH_EVERYONE);
	parent->objects.push_back(this);
}

PlayerShape::PlayerShape(PhysicsWorld* parent, Real radius, Real cone_height, Real torso_height, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	this->parent = parent;
	// Here we use the convention that sizes are the total thicknesses, but Bullet uses half-extents.
#if 0
	shape = new btCompoundShape();
	// According to this post:
	// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=1699
	// the cone's center is at half its height, so I'll take this as received wisdom for now.
	btTransform bottomCone(btQuaternion(btVector3(1.0, 0.0, 0.0), M_PI/2.0), btVector3(0.0, - cone_height/2.0 - torso_height/2.0, 0.0));
	btTransform topCone   (btQuaternion(btVector3(1.0, 0.0, 0.0), 0.0),      btVector3(0.0,   cone_height/2.0 + torso_height/2.0, 0.0));
	// FIXME: I lose the pointers to the children, so they'll be hard to free later, when I want to not leak memory.
	reinterpret_cast<btCompoundShape*>(shape)->addChildShape(bottomCone, new btConeShape(radius, cone_height));
	reinterpret_cast<btCompoundShape*>(shape)->addChildShape(topCone, new btConeShape(radius, cone_height));
#else
	shape = new btBoxShape(btVector3(0.3, 0.3, 0.3));
#endif

	btVector3 rotation_axis(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);
	auto rotation = btQuaternion(rotation_axis, rotation_quaternion[3]);
	motionState = new btDefaultMotionState(btTransform(rotation, btVector3(position[0], position[1], position[2])));
	btVector3 inertia(0, 0, 0);
	// XXX: Warning: I make it so that all capsules have infinite moment of inertia!
//	shape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
	rigidBody = new btRigidBody(rigidBodyCI);
	//	rigidBody->setFriction(2.0);
	// Add ourselves into the parent's simulation.
	parent->dynamicsWorld->addRigidBody(rigidBody, collision_group, COLLIDE_WITH_EVERYONE);
	parent->objects.push_back(this);
}

BvhTriangleMesh::BvhTriangleMesh(PhysicsWorld* parent, Real* position, Real* rotation_quaternion, int triangle_count, Real* triangles, int collision_group) {
	this->parent = parent;
	// Load the mesh into a Bullet data structure.
	mesh = new btTriangleMesh();
	for (int i = 0; i < triangle_count; i++) {
		btVector3 vertex0(triangles[0], triangles[1], triangles[2]);
		btVector3 vertex1(triangles[3], triangles[4], triangles[5]);
		btVector3 vertex2(triangles[6], triangles[7], triangles[8]);
		// If setting removeDuplicateVertices=true starts to become a performance issue on load, revisit this.
		mesh->addTriangle(vertex0, vertex1, vertex2, true);
		triangles += 9;
	}
	shape = new btBvhTriangleMeshShape(mesh, true);
	btVector3 rotation_axis(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2]);
	auto rotation = btQuaternion(rotation_axis, rotation_quaternion[3]);
	motionState = new btDefaultMotionState(btTransform(rotation, btVector3(position[0], position[1], position[2])));
	btVector3 inertia(0, 0, 0);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0.0, motionState, shape, inertia);
	rigidBody = new btRigidBody(rigidBodyCI);
//	rigidBody->setFriction(2.0);
	// Add ourselves into the parent's simulation.
	parent->dynamicsWorld->addRigidBody(rigidBody, collision_group, COLLIDE_WITH_EVERYONE);
	parent->objects.push_back(this);
}

extern "C" EXPORT PhysicsWorld* PhysicsWorld_new(void) {
	return new PhysicsWorld();
}

extern "C" EXPORT void PhysicsWorld_step(PhysicsWorld* w, Real dt) {
	w->step(dt);
}

extern "C" EXPORT int PhysicsWorld_rayCast(PhysicsWorld* w, Real* start, Real* end, Real* hit, Real* normal, PhysicsObject** hit_object, int collision_mask) {
	btVector3 vec_start(start[0], start[1], start[2]);
	btVector3 vec_end(end[0], end[1], end[2]);
	btVector3 result, normal_vec;
	const btCollisionObject* hit_collision_object;
	if (w->rayCast(vec_start, vec_end, result, normal_vec, hit_collision_object, collision_mask)) {
		hit[0] = result.getX();
		hit[1] = result.getY();
		hit[2] = result.getZ();
		normal[0] = normal_vec.getX();
		normal[1] = normal_vec.getY();
		normal[2] = normal_vec.getZ();
		// Figure out which PhysicsObject has the given btCollisionObject.
		for (auto obj : w->objects)
			if (obj->rigidBody == hit_collision_object)
				*hit_object = obj;
		return 1;
	}
	return 0;
}

extern "C" EXPORT int PhysicsWorld_checkForContact(PhysicsWorld* w, PhysicsObject* a, PhysicsObject* b, int collision_mask) {
	return w->checkForContact(a, b, collision_mask);
}

extern "C" EXPORT int PhysicsWorld_convexSweepTest(PhysicsWorld* w, Real radius, Real* start, Real* end, Real* hit, Real* normal, PhysicsObject** hit_object, int collision_mask) {
	btVector3 vec_start(start[0], start[1], start[2]);
	btVector3 vec_end(end[0], end[1], end[2]);
	btVector3 result, normal_vec;
	const btCollisionObject* hit_collision_object;
	if (w->convexSweepTest(radius, vec_start, vec_end, result, normal_vec, hit_collision_object, collision_mask)) {
		hit[0] = result.getX();
		hit[1] = result.getY();
		hit[2] = result.getZ();
		normal[0] = normal_vec.getX();
		normal[1] = normal_vec.getY();
		normal[2] = normal_vec.getZ();
		// Figure out which PhysicsObject has the given btCollisionObject.
		for (auto obj : w->objects)
			if (obj->rigidBody == hit_collision_object)
				*hit_object = obj;
		return 1;
	}
	return 0;
}

extern "C" EXPORT int PhysicsWorld_listAllCollidingPairs(PhysicsWorld* w, PhysicsObject*** pairs) {
	set<pair<PhysicsObject*, PhysicsObject*>>& colliding_pairs = *w->listAllCollidingPairs();
	int pair_count = colliding_pairs.size();
	// Convert this fancy STL datum into something more conducive to C.
	// In this case, pairs are stored adjacently in an array of length 2n.
	PhysicsObject** array = new PhysicsObject*[2 * pair_count];
	int i = 0;
	for (const pair<PhysicsObject*, PhysicsObject*>& p : colliding_pairs) {
		array[2*i]     = p.first;
		array[2*i + 1] = p.second;
		i++;
	}
	delete &colliding_pairs;
	*pairs = array;
	return pair_count;
}

extern "C" EXPORT void delete_colliding_pairs_list(PhysicsObject** pairs) {
	delete[] pairs;
}

extern "C" EXPORT Box* Box_new(PhysicsWorld* parent, Real* bounds, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	return new Box(parent, bounds, position, rotation_quaternion, mass, collision_group);
}

extern "C" EXPORT Sphere* Sphere_new(PhysicsWorld* parent, Real radius, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	return new Sphere(parent, radius, position, rotation_quaternion, mass, collision_group);
}

extern "C" EXPORT ConvexHull* ConvexHull_new(PhysicsWorld* parent, Real* position, Real* rotation_quaternion, int point_count, Real* points, Real mass, int collision_group) {
	return new ConvexHull(parent, position, rotation_quaternion, point_count, points, mass, collision_group);
}

extern "C" EXPORT Capsule* Capsule_new(PhysicsWorld* parent, Real radius, Real height, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	return new Capsule(parent, radius, height, position, rotation_quaternion, mass, collision_group);
}

extern "C" EXPORT PlayerShape* PlayerShape_new(PhysicsWorld* parent, Real radius, Real cone_height, Real torso_height, Real* position, Real* rotation_quaternion, Real mass, int collision_group) {
	return new PlayerShape(parent, radius, cone_height, torso_height, position, rotation_quaternion, mass, collision_group);
}

extern "C" EXPORT BvhTriangleMesh* BvhTriangleMesh_new(PhysicsWorld* parent, int triangle_count, Real* triangles, Real* position, Real* rotation_quaternion, int collision_group) {
//	Real position[] = {0.0, 0.0, 0.0};
//	Real rotation_quaternion[] = {1.0, 0.0, 0.0, 0.0};
	return new BvhTriangleMesh(parent, position, rotation_quaternion, triangle_count, triangles, collision_group);
}

extern "C" EXPORT void PhysicsObject_removeFromWorld(PhysicsObject* obj) {
	obj->removeFromWorld();
}

extern "C" EXPORT void PhysicsObject_getPos(PhysicsObject* obj, Real* xyz) {
	obj->getPos(xyz);
}

extern "C" EXPORT void PhysicsObject_setPos(PhysicsObject* obj, Real x, Real y, Real z) {
	obj->setPos(x, y, z);
}

extern "C" EXPORT void PhysicsObject_getAxisAngle(PhysicsObject* obj, Real* axis_angle) {
	obj->getAxisAngle(axis_angle);
}

extern "C" EXPORT void PhysicsObject_setAxisAngle(PhysicsObject* obj, Real x, Real y, Real z, Real t) {
	obj->setAxisAngle(x, y, z, t);
}

extern "C" EXPORT void PhysicsObject_setPosAxisAngle(PhysicsObject* obj, Real xx, Real yy, Real zz, Real x, Real y, Real z, Real t) {
	obj->setPosAxisAngle(xx, yy, zz, x, y, z, t);
}

extern "C" EXPORT void PhysicsObject_getLinearVelocity(PhysicsObject* obj, Real* xyz) {
	obj->getLinearVelocity(xyz);
}

extern "C" EXPORT void PhysicsObject_setLinearVelocity(PhysicsObject* obj, Real vx, Real vy, Real vz) {
	Real velocity[3] = {vx, vy, vz};
	obj->setLinearVelocity(velocity);
}

extern "C" EXPORT void PhysicsObject_getAngularVelocity(PhysicsObject* obj, Real* velocity) {
	obj->getAngularVelocity(velocity);
}

extern "C" EXPORT void PhysicsObject_setAngularVelocity(PhysicsObject* obj, Real vx, Real vy, Real vz) {
	Real velocity[3] = {vx, vy, vz};
	obj->setAngularVelocity(velocity);
}

extern "C" EXPORT void PhysicsObject_setLocalScaling(PhysicsObject* obj, Real vx, Real vy, Real vz) {
	Real scaling[3] = {vx, vy, vz};
	obj->setLocalScaling(scaling);
}

extern "C" EXPORT void PhysicsObject_convertIntoReferenceFrame(PhysicsObject* obj) {
	obj->convertIntoReferenceFrame();
}

extern "C" EXPORT void PhysicsObject_applyForce(PhysicsObject* obj, Real x, Real y, Real z) {
	obj->applyForce(x, y, z);
}

extern "C" EXPORT void PhysicsObject_applyCentralImpulse(PhysicsObject* obj, Real x, Real y, Real z) {
	obj->applyCentralImpulse(x, y, z);
}

extern "C" EXPORT void PhysicsObject_setLinearFactor(PhysicsObject* obj, Real x, Real y, Real z) {
	obj->setLinearFactor(x, y, z);
}

extern "C" EXPORT void PhysicsObject_setAngularFactor(PhysicsObject* obj, Real x, Real y, Real z) {
	obj->setAngularFactor(x, y, z);
}

extern "C" EXPORT void PhysicsObject_setGravity(PhysicsObject* obj, Real x, Real y, Real z) {
	obj->setGravity(x, y, z);
}

extern "C" EXPORT void PhysicsObject_setKinematic(PhysicsObject* obj, int is_kinematic) {
	obj->setKinematic(is_kinematic);
}

