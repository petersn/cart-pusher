#!/usr/bin/python
# Game simulation for both clients and the server for cart-pusher.

import os, math, struct, json, base64, time
import numpy as np
import compgeom
import link
import render
from config import *

COLLISION_SOLID  = 1 << 0
COLLISION_PLAYER = 1 << 1
COLLISION_ENEMY  = 1 << 2

class GameSimulation:
	SERVER_TIME_ALPHA = 0.1

	def __init__(self, is_server_side=True):
		self.player_list = {}
		self.entities = {}
		self.physics = link.PhysicsWorld()
		self.total_game_time = 0.0
		self.next_entity_id = 1000
		self.server_time_delta_estimate = None
		self.is_server_side = is_server_side

	def step(self, dt):
		# XXX: Floating point accumulation issues here?
		self.total_game_time += dt

		# Update Bullet's state for the universe.
		self.physics.step(dt)

		# Build a table of which entities have which geoms.
		# NB: Thus, specialness of having a member named geom is baked in here.
		geom_to_entity = {}
		for entity in self.entities.itervalues():
			if hasattr(entity, "geom"):
				geom_to_entity[entity.geom] = entity
		# Get a table of the colliding pairs of geoms.
		colliding_geoms = self.physics.listAllCollidingPairs()
		# Dispatch collision events to various entities.
		for a, b in colliding_geoms:
			# Both geoms have to be tracked in entities for the collision to count.
			ent_a = geom_to_entity.get(a, None)
			ent_b = geom_to_entity.get(b, None)
			if ent_a == None or ent_b == None:
				continue
			# Dispatch!
			ent_a.collision(ent_b, dt)
			ent_b.collision(ent_a, dt)

		# Dispatch the step to the various entities.
		for entity in self.entities.values():
#			assert entity._was_instantiated, "BUG: Forgot to call instantiate_tied in Entity subclass's __init__"
			entity.step(dt)
#			# Kill entities with zero or less hp.
#			if entity.damageable and entity.hp <= 0:
#				entity.kill()

	def draw(self):
		for entity in self.entities.itervalues():
			entity.draw()

	def new_entity_id(self):
		self.next_entity_id += 1
		return self.next_entity_id - 1

	def add_entity(self, constructor, initialization_args, specific_ent_id=None):
		print "Creating entity:", constructor.serialization_key, initialization_args, specific_ent_id
		# XXX: Maybe clean up the signature here? I don't like how things leak.
		# Specifically, specific_ent_id leaks into entity.init, and isn't apparent from the signature.
		entity = constructor()
		entity.sim = self
		entity.ent_id = specific_ent_id if specific_ent_id != None else self.new_entity_id()
		entity.initialization_args = initialization_args
		entity.init(*initialization_args)
		# Specially sort out players.
#		if isinstance(entity, Player):
#			self.player_list[entity.ent_id] = entity
		self.entities[entity.ent_id] = entity
		return entity

	def remove_entity(self, ent_id):
		if ent_id in self.entities:
			self.entities.pop(ent_id)

	def get_ent(self, filter_function):
		for entity in self.entities.itervalues():
			if filter_function(entity):
				return entity

	def line_of_sight(self, entity1, entity2, collision_mask=COLLISION_SOLID, max_range=float("inf")):
		xyz1 = entity1.get_xyz()
		xyz2 = entity2.get_xyz()
		object_distance = compgeom.l2(xyz1, xyz2)
		if object_distance > max_range:
			return False
		result = self.physics.rayCast(xyz1, xyz2, collision_mask)
		# If there's no hit, then we have line of sight.
		if not result:
			return True
		distance_to_first_hit = compgeom.l2(xyz1, result.xyz)
		# Otherwise, check to see if the ray is longer than the objects' separation.
		if distance_to_first_hit >= object_distance:
			return True
		return False

	def serialize_game_state(self):
		desc = {
			"vars": {
				"total_game_time": self.total_game_time,
				"next_entity_id": self.next_entity_id,
			},
			"ents": {},
		}
		for ent_id, entity in self.entities.iteritems():
			desc["ents"][ent_id] = {
				"type": entity.serialization_key,
				"init_args": entity.initialization_args,
				"patch": base64.b64encode(entity.serialize_patch()),
			}
		return STATE_COMPRESS(json.dumps(desc))

	def serialize_game_patches(self):
		now = time.time()
		s = [struct.pack("<d", now)]
		for entity in self.entities.itervalues():
			s.append(struct.pack("<I", entity.ent_id))
			s.append(entity.serialize_patch())
		return "".join(s)

	def apply_serialized_state(self, desc):
		# XXX: Security implications of this line?
		# It's not like we have Javascript-style __proto__ issues, but...
		self.total_game_time = (1.0 - self.SERVER_TIME_ALPHA) * self.total_game_time + self.SERVER_TIME_ALPHA * desc["vars"]["total_game_time"]
		self.next_entity_id = desc["vars"]["next_entity_id"]
		# Mark each entity for culling.
		for entity in self.entities.itervalues():
			entity.do_cull = True
		# We'll diff our entity list from the descriptor given.
		for ent_id, entity_desc in desc["ents"].iteritems():
			ent_id = int(ent_id)
			# If we already have the entity then perform a standard patch.
			# TODO: Re-evaluate if I want to do this here at all.
			if ent_id not in self.entities:
				constructor = serialization_key_table[entity_desc["type"]]
				self.add_entity(constructor, entity_desc["init_args"], ent_id)
				# NB: You can un-indent the rest of this if-block to get patches
				# applied from the JSON state even to existing objects.
				# I'm currently not doing this because of the lack of anti-jitter.
				# Assert that the entity types match.
				# TODO: Resync properly if they don't.
				entity = self.entities[ent_id]
				assert entity.serialization_key == entity_desc["type"]
				entity.apply_patch(entity_desc["patch"].decode("base64"), 0)
			entity = self.entities[ent_id]
			entity.do_cull = False
		# We now find the entities that we need to cull.
		self.entities = {k: v for k, v in self.entities.iteritems() if not v.do_cull}

	def apply_serialized_patches(self, payload):
#		# Mark every entity for culling -- they'll get cleared as we see them in the payload.
#		for entity in self.entities.itervalues():
#			entity.do_cull = True
		# Iterate over update.
		server_time, = struct.unpack("<d", payload[:8])
		now = time.time()
		if self.server_time_delta_estimate is None:
			self.server_time_delta_estimate = server_time - now
			print "Initial clock delta:", self.server_time_delta_estimate
		jitter = server_time - now - self.server_time_delta_estimate
		self.server_time_delta_estimate = (1 - self.SERVER_TIME_ALPHA) * self.server_time_delta_estimate + self.SERVER_TIME_ALPHA * (server_time - now)
		i = 8
		while i < len(payload):
			ent_id, = struct.unpack("<I", payload[i:i+4])
			i += 4
			# If we haven't heard of this object then create it.
			if ent_id not in self.entities:
				print "Invalid ent_id, skipping rest of payload:", ent_id
#				print "Resync will happen at next large update."
				return
#				print "Making a guy:", ent_id, ent_type
#				self.entities[ent_id] = serialization_key_table[ent_type]()
			# Apply our patch, and update the counter.
			i = self.entities[ent_id].apply_patch(payload, i, jitter)

class Entity:
	def serialize_patch(self):
		return ""

	def apply_patch(self, payload, i, jitter=0.0):
		return i

	def collision(self, other, dt):
		pass

	def draw(self):
		pass

	def step(self, dt):
		pass

	# Some generic helper functions.

	def get_xyz(self):
		return np.array(self.geom.getPos())

	def destroy(self):
		self.sim.remove_entity(self.ent_id)

	def get_elevation(self, collision_mask=COLLISION_SOLID, corrections=0.0):
		MAX_ELEVATION = 10000
		xyz = self.get_xyz()
		target = self.sim.physics.rayCast(xyz, xyz - np.array([0, 0, MAX_ELEVATION]), collision_mask=collision_mask)
		# Determine the elevation of our player.
		elevation = xyz[2] - target.xyz[2] - corrections if target != None else MAX_ELEVATION
		return elevation

	def player_in_los(self, max_range=1000.0):
		for player in self.sim.entities.itervalues():
			if not isinstance(player, Player):
				continue
			if self.sim.line_of_sight(self, player, max_range=self.max_aggro_range):
				return player

class EnemyMixin:
	pass

class GeomOnlyPatchMixin:
	def serialize_patch(self):
		return struct.pack("<6f", *self.geom.getPos() + self.geom.getLinearVelocity())

	def apply_patch(self, patch, i, jitter=0.0):
		pos  = struct.unpack("<3f", patch[i:i+12])
		velo = struct.unpack("<3f", patch[i+12:i+24])
		if USE_JITTER_CORRECTION:
			pos = np.array(pos) - np.array(velo) * jitter
#		print self, "patch:", pos, velo
		self.geom.setPos(pos)
		self.geom.setLinearVelocity(velo)
		return i + 24

class TerrainSegment(Entity):
	serialization_key = "t"

	def init(self, xyz, segment_type):
		self.model = render.get_model(segment_type)
		self.geom = link.BvhTriangleMesh(self.sim.physics, "stone", self.model.obj.triangles, self.model.obj.triangles_texture_coords, xyz, (1, 0, 0, 0))

	def draw(self):
		link.glPushMatrix()
		self.geom.convertIntoReferenceFrame()
		self.model.render()
		link.glPopMatrix()

class Block(Entity):
	serialization_key = "b"
	mass = 1.0

	def init(self, xyz):
#		self.model = render.get_model("block")
		self.geom = link.Box(self.sim.physics, "stone", (1.0, 1.0, 1.0), xyz, (1, 0, 0, 0), self.mass)

	def serialize_patch(self):
		s = []
		s.append(struct.pack("<3f", *self.geom.getPos()))
		s.append(struct.pack("<3f", *self.geom.getLinearVelocity()))
		s.append(struct.pack("<4f", *self.geom.getAxisAngle()))
		# TODO: Also get the angular velocity and save that.
		return "".join(s)

	def apply_patch(self, patch, i, jitter=0.0):
		pos  = struct.unpack("<3f", patch[i:i+12])
		velo = struct.unpack("<3f", patch[i+12:i+24])
		axis_angle = struct.unpack("<4f", patch[i+24:i+40])
		if USE_JITTER_CORRECTION:
			pos = np.array(pos) - np.array(velo) * jitter
			# TODO: Once the angular velocity is encoded, also jitter correct axis_angle here.
		self.geom.setPos(pos)
		self.geom.setLinearVelocity(velo)
		self.geom.setAxisAngle(axis_angle)
		return i + 40

	def draw(self):
		obj = self.geom
		x, y, z = obj.getPos()
		t1, t2, t3 = obj.bounds[0]/2.0, obj.bounds[1]/2.0, obj.bounds[2]/2.0
		link.glPushMatrix()
		obj.convertIntoReferenceFrame()
		bounds = [-t1, +t1, -t2, +t2, -t3, +t3]
		link.draw_box(bounds, obj.texture)
		link.glPopMatrix()

class Jumper(Entity, GeomOnlyPatchMixin, EnemyMixin):
	serialization_key = "j"
	max_aggro_range = 50.0
	radius = 0.7
	gl_scale = 1.0
	max_hp = 20
	damage = 40
	mass = 2.0

	leap_coef = 0.5
	max_leap_velocity = 20.0
	leap_vertical_component = 10

	def init(self, xyz):
		self.geom = link.Sphere(self.sim.physics, "stone", self.radius, xyz, (1, 0, 0, 0), self.mass, collision_group=COLLISION_PLAYER)
		self.aggro_target = -1
		self.jump_cooldown = 0.0
		self.hp = self.max_hp
		self.lifespan = 10.0

	def serialize_patch(self):
		base_patch = GeomOnlyPatchMixin.serialize_patch(self)
		base_patch += struct.pack("<fi", self.jump_cooldown, self.aggro_target)
		return base_patch

	def apply_patch(self, patch, i, jitter=0.0):
		i = GeomOnlyPatchMixin.apply_patch(self, patch, i, jitter)
		self.jump_cooldown, self.aggro_target = struct.unpack("<fi", patch[i:i+8])
		return i + 8

	def step(self, dt):
		player = self.player_in_los(max_range=self.max_aggro_range)
		if player and self.aggro_target == -1:
			self.aggro_target = player.ent_id

		self.jump_cooldown = max(0.0, self.jump_cooldown - dt)

		if self.aggro_target != -1 and self.aggro_target in self.sim.entities:
			player = self.sim.entities[self.aggro_target]
			assert isinstance(player, Player), "BUG BUG BUG %i" % self.aggro_target
			self.elevation = self.get_elevation(corrections=self.radius)
			if self.elevation < 0.4 and self.jump_cooldown == 0.0:
				self.jump_towards_entity(player)

		# Server side maximum lifespan.
		self.lifespan -= dt
		if self.lifespan <= 0.0 and self.sim.is_server_side:
			self.destroy()

	def jump_towards_entity(self, entity):
		self.jump_cooldown = 0.5
#		print "Performing jump."
		vector = entity.get_xyz() - self.get_xyz()
#		vector[2] = 0.0
#		vector /= max(1.0, np.linalg.norm(vector))
#		vector *= 10.0
		vector *= self.leap_coef
		vector[2] = max(self.leap_vertical_component, vector[2])
		# Make sure they don't jump too fast.
		if np.linalg.norm(vector) > self.max_leap_velocity:
			vector *= self.max_leap_velocity / np.linalg.norm(vector)
#		vector[2] = 10.0
		self.geom.setLinearVelocity(vector)
#		self.geom.applyForce(*(vector * 20))

	def draw(self):
		link.glPushMatrix()
		self.geom.convertIntoReferenceFrame()
		link.glScalef(self.gl_scale, self.gl_scale, self.gl_scale)
		render.get_model("jumper").render()
		link.glPopMatrix()

	def collision(self, other, dt):
		if isinstance(other, Player):
			print "Hit the player!"
			self.destroy()
			other.deal_damage(self.damage)

class BigJumper(Jumper):
	serialization_key = "J"
	max_aggro_range = 50.0
	radius = 0.7 * 2.0
	gl_scale = 2.0
	max_hp = 50
	damage = 55
	mass = 8.0

	leap_coef = 0.9
	max_leap_velocity = 20.0
	leap_vertical_component = 8

class EnemySpawner(Entity):
	serialization_key = "s"
	max_aggro_range = 40.0

	def init(self, xyz, enemy_type, interval):
		self.xyz = xyz
		self.enemy_type = enemy_type
		self.interval = interval
		self.spawn_cooldown = 0.0

	def get_xyz(self):
		return self.xyz

	def step(self, dt):
		self.spawn_cooldown = max(0.0, self.spawn_cooldown - dt)

		if self.spawn_cooldown == 0.0 and self.player_in_los(max_range=self.max_aggro_range):
			self.sim.add_entity(serialization_key_table[self.enemy_type], [self.xyz])
			self.spawn_cooldown = self.interval

class RandomlyGeneratedTerrain(TerrainSegment):
	serialization_key = "r"

	def init(self, xyz, seed):
		import level_generator
		self.lg = level_generator.LevelGenerator(seed)
		self.lg.generate_triangles()
		self.geom = link.BvhTriangleMesh(self.sim.physics, "stone", self.lg.triangles, self.lg.triangles_texture_coords, xyz, (1, 0, 0, 0))
		self.arrays = link.pack_triangles(self.lg.triangles, self.lg.triangles_texture_coords)
		self.texture = link.get_texture(os.path.join("data", "stone.png"))

		# We now make everything we need.

	def draw(self):
		link.glPushMatrix()
		self.geom.convertIntoReferenceFrame()
		link.draw_triangles(self.arrays, self.texture.index)
		link.glPopMatrix()

class Cart(GeomOnlyPatchMixin, Entity):
	serialization_key = "c"
	mass = 1000.0
	cart_speed = 5.0
	contest_distance = 8.0

	def init(self, xyz):
		self.geom = link.Box(self.sim.physics, "stone", (2.0, 4.0, 1.5), xyz, (0, 0, -1, math.pi/2), self.mass)
		self.geom.setAngularFactor(0.0, 0.0, 0.0)
		self.geom.setGravity(0.0, 0.0, 0.0)
		self.push_state = "idle"
		self.accumulated_movement = 30.0
		self.current_speed = 0.0
		self.current_theta = 0.0

	def serialize_patch(self):
		return struct.pack("<d", self.accumulated_movement)

	def apply_patch(self, patch, i, jitter=0.0):
		self.accumulated_movement, = struct.unpack("<d", patch[i:i+8])
		if USE_JITTER_CORRECTION:
			self.accumulated_movement -= self.current_speed * jitter
		return i + 8

	def draw(self):
		link.glPushMatrix()
		self.geom.convertIntoReferenceFrame()
		render.get_model("cart").render()
		link.glPopMatrix()

	def _draw(self):
		obj = self.geom
		x, y, z = obj.getPos()
		t1, t2, t3 = obj.bounds[0]/2.0, obj.bounds[1]/2.0, obj.bounds[2]/2.0
		link.glPushMatrix()
		obj.convertIntoReferenceFrame()
		bounds = [-t1, +t1, -t2, +t2, -t3, +t3]
		link.draw_box(bounds, obj.texture)
		link.glPopMatrix()

	def step(self, dt):
#		# Position the cart based on the total game time.
#		self.geom.setPos((self.sim.total_game_time * self.cart_speed, 0, 1.5))
#		self.geom.setLinearVelocity((0, 0, 0))
		# Figure out if there are any players near the cart.
		self.push_state = "idle"
		xyz = self.get_xyz()
		for entity in self.sim.entities.itervalues():
			if compgeom.l2(xyz, entity.get_xyz()) > self.contest_distance:
				continue
			if isinstance(entity, Player):
				self.push_state = "pushing"
			if isinstance(entity, EnemyMixin):
				self.push_state = "contested"
				break
		multiplier = {"idle": 0, "pushing": 1, "contested": 0}[self.push_state]
		self.current_speed = multiplier * self.cart_speed
		self.accumulated_movement += self.current_speed * dt
		#self.geom.setLinearVelocity((multiplier * self.cart_speed, 0, 0))
		level_gen = self.sim.get_ent(lambda x: isinstance(x, RandomlyGeneratedTerrain)).lg
		new_xyz, new_theta = level_gen.get_xyz_theta_on_path(self.accumulated_movement)
		new_xyz += np.array([0, 0, 1.0])
		self.current_theta = 0.95 * self.current_theta + 0.05 * new_theta
		self.geom.setPos(new_xyz)
		self.geom.setAxisAngle((0, 0, -1, math.pi/2 - self.current_theta))

class Player(GeomOnlyPatchMixin, Entity):
	move_speed = 10.0
	move_stiffness = 15.0 * 60.0
	jump_velocity = 8.0
	jump_count = 1
	serialization_key = "p"
	mass = 1.0
	radius = 0.5
	eye_height = 0.75
	is_actively_controlled_player = False

	def init(self, xyz, name):
		self.name = name
		self.geom = link.Sphere(self.sim.physics, "stone", self.radius, xyz, (1, 0, 0, 0), self.mass, collision_group=COLLISION_PLAYER)
		self.geom.setAngularFactor(0.0, 0.0, 0.0)
		self.input_motion_vector = (0.0, 0.0)
		self.facing = 0.0
		self.tilt = 0.0
		self.jumps_available = 0
		self.hp = 100

#	def init_from_state(self, desc):
#		self.name = desc["name"]

#	def serialize_state(self):
#		return {"name": self.name}

	def serialize_patch(self):
		s = []
		s.append(struct.pack("<3f", *self.geom.getPos()))
		s.append(struct.pack("<3f", *self.geom.getLinearVelocity()))
		s.append(struct.pack("<2f", *self.input_motion_vector))
		s.append(struct.pack("<hh", self.jumps_available, self.hp))
		return "".join(s)

	def apply_patch(self, patch, i, jitter=0.0):
		pos  = struct.unpack("<3f", patch[i:i+12])
		velo = struct.unpack("<3f", patch[i+12:i+24])
		motion_vector = struct.unpack("<2f", patch[i+24:i+32])
		self.jumps_available, self.hp = struct.unpack("<hh", patch[i+32:i+36])
		if USE_JITTER_CORRECTION:
			pos = np.array(pos) - np.array(velo) * jitter
#		print self, "patch:", pos, velo
		self.geom.setPos(pos)
		self.geom.setLinearVelocity(velo)
		if not self.is_actively_controlled_player:
			self.input_motion_vector = motion_vector
		return i + 4 * 8 + 2 + 2

	def draw(self):
		if self.is_actively_controlled_player:
			return
		link.glPushMatrix()
		self.geom.convertIntoReferenceFrame()
		render.get_model("player_model").render()
		link.glPopMatrix()

	def deal_damage(self, amount):
		self.hp -= amount

	def step(self, dt):
		# XXX: Not framerate independent yet!
		self.apply_forces(dt)
		self.elevation = self.get_elevation(collision_mask=COLLISION_SOLID | COLLISION_ENEMY, corrections=self.radius)
		if self.elevation < 0.4:
			self.jumps_available = self.jump_count
		else:
			# Prevent the player from getting two full jumps if they walk off an edge.
			self.jumps_available = min(self.jumps_available, self.jump_count - 1)

	def respawn(self):
		# Respawn over the cart.
		cart = self.sim.get_ent(lambda ent: isinstance(ent, Cart))
		self.geom.setPos(cart.get_xyz() + np.array([0, 0, 2]))
#		self.geom.setPos(np.array([0, 0, 2]))
		self.geom.setLinearVelocity((0, 0, 0))

	def adjust_camera(self, horizontal, vertical):
		self.facing += horizontal
		self.tilt += vertical
		self.tilt = min(math.pi/2, max(-math.pi/2, self.tilt))

	def try_to_jump(self):
		if self.jumps_available > 0:
			velo = self.geom.getLinearVelocity()
			# Rules for jumping (corresponding to the three min/max calls:
			# 0) By default your new vertical speed is self.jump_velocity.
			# 1) (innermost max) It never slows you down if you were already moving up.
			# 2) (min) It never adds more than jump_velocity*2 to your speed (in case you were falling really fast), unless:
			# 3) (outermost max) You're falling super duper fast, in which case you still get a half height (1/\sqrt2 velo) jump.
			velo[2] = max(min(max(velo[2], self.jump_velocity), velo[2] + self.jump_velocity * 2), self.jump_velocity * (2**-0.5))
			self.geom.setLinearVelocity(velo)
			self.jumps_available -= 1

	def get_facing_vector(self):
		facing_sin, facing_cos = math.sin(self.facing), math.cos(self.facing)
		tilt_sin, tilt_cos = math.sin(self.tilt), math.cos(self.tilt)
		x = facing_sin * tilt_cos
		y = facing_cos * tilt_cos
		z = -tilt_sin
		return x, y, z

	def get_motion_xy(self):
		right, forward = self.input_motion_vector
		# Normalize the right, forward vector to have a maximum length of one.
		# Otherwise the fastest way to move would be to hold down forwards and left, and turn at a 45 degree angle.
		norm = (right**2 + forward**2)**0.5
		if norm > 1:
			right /= norm
			forward /= norm
		sin, cos = math.sin(self.facing), math.cos(self.facing)
		x = sin * right + cos * forward
		y = cos * right - sin * forward
		return x, y

	def apply_forces(self, dt):
		x, y = self.get_motion_xy()
		vel = self.geom.getLinearVelocity()
		# WARNING: The value 10000 is the maximum height we can be over terrain before the ray is missed!
		MAX_ELEVATION = 10000
		dx, dy = self.move_speed*x - vel[0], self.move_speed*y - vel[1]
		self.geom.applyForce(self.move_stiffness * dx * dt, self.move_stiffness * dy * dt, 0)

serialization_key_table = {}
for cls in globals().values():
	if hasattr(cls, "serialization_key"):
		assert cls.serialization_key not in serialization_key_table, "Collision in serialization_keys!"
		serialization_key_table[cls.serialization_key] = cls

def initialize_game(sim):
	# Create a cart at the origin.
	sim.add_entity(Cart, [(0, 0, 1.0)])
	sim.add_entity(EnemySpawner, [(20, 0, 2), "j", 5.0])
#	sim.add_entity(BigJumper, [(21, 1, 10)])
#	for i in xrange(50):
#		sim.add_entity(Jumper, [(20.0 + i * 0.05, i * 0.05, 1.0 + i * 2.0)])

#	sim.add_entity(TerrainSegment, [(0, 0, 0), "basic_units"])
	sim.add_entity(RandomlyGeneratedTerrain, [(0, 0, 0), 10])

