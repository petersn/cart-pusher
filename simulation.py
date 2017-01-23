#!/usr/bin/python
# Game simulation for both clients and the server for cart-pusher.

import os, math, struct, json, base64, time
import numpy as np
import compgeom
import link
import render
import characters
from config import *

COLLISION_SOLID  = 1 << 0
COLLISION_PLAYER = 1 << 1
COLLISION_ENEMY  = 1 << 2

class GameSimulation:
	SERVER_TIME_ALPHA = 0.1

	def __init__(self, is_server_side=True):
		self.entities = {}
		self.effects = {}
		self.physics = link.PhysicsWorld()
		self.total_game_time = 0.0
		self.next_entity_id = 1000
		self.server_time_delta_estimate = None
		self.is_server_side = is_server_side
		self.previous_patch_from_entity = {}
		self.client_known_ent_ids = set()

		# These are server-side configuration values.
		self.difficulty = 0
		self.debug = 0

		# These are game-play critical values.
		self.current_checkpoint_value = 0

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

		# Dispatch to effects.
		for effect in self.effects.values():
			effect.step(dt)

	def draw(self):
		for entity in self.entities.itervalues():
			entity.draw()
		for effect in self.effects.itervalues():
			effect.draw()

	def new_entity_id(self):
		self.next_entity_id += 1
		return self.next_entity_id - 1

	def add_entity(self, constructor, initialization_args, specific_ent_id=None):
		assert issubclass(constructor, Entity), "Tried to add_entity %r" % (constructor,)
		# We don't construct client-side only entities on the server.
		if constructor.client_side_only_entity and self.is_server_side:
			return
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

	def remove_effect(self, effect):
		if effect in self.effects:
			self.effects.pop(effect)

	def get_ent(self, filter_function):
		for entity in self.entities.itervalues():
			if filter_function(entity):
				return entity

	def add_effect(self, constructor, initialization_args):
		effect = constructor()
		effect.sim = self
		effect.initialization_Args = initialization_args
		effect.init(*initialization_args)
		self.effects[effect] = effect
		return effect

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
				# Send server-side configuration.
				"difficulty": self.difficulty,
				"debug": self.debug,
			},
			"ents": {},
		}
		def make_kosher_for_json(x):
			if isinstance(x, np.ndarray):
				return list(x)
			return x
		for ent_id, entity in self.entities.iteritems():
			desc["ents"][ent_id] = {
				"type": entity.serialization_key,
				"init_args": map(make_kosher_for_json, entity.initialization_args),
				"patch": base64.b64encode(entity.serialize_patch()),
			}
		return STATE_COMPRESS(json.dumps(desc))

	def serialize_game_patches(self):
		now = time.time()
		s = [struct.pack("<d", now)]
		for entity in self.entities.itervalues():
			is_new = entity.ent_id not in self.client_known_ent_ids
			new_patch = entity.serialize_patch()
			if new_patch == self.previous_patch_from_entity.get(entity.ent_id, None):
				continue
			self.previous_patch_from_entity[entity.ent_id] = new_patch
			# If the entity is new then encode one billion plus the entity ID, to signal newness.
			encoded_ent_id = entity.ent_id + ENTITY_CREATION_OFFSET if is_new else entity.ent_id
			s.append(struct.pack("<i", encoded_ent_id))
			# Further, if this is a brand new entity, then send a special creation message.
			if is_new:
				print "Encoding new:", encoded_ent_id
				json_payload = json.dumps(entity.initialization_args)
				s.append(entity.serialization_key)
				s.append(struct.pack("<I", len(json_payload)))
				s.append(json_payload)
			s.append(new_patch)
		# Finally, encode deaths from the previous frame.
		for ent_id in self.client_known_ent_ids:
			if ent_id not in self.entities:
				print "Encoding death:", ent_id
				s.append(struct.pack("<i", -ent_id))
		self.client_known_ent_ids = set(entity.ent_id for entity in self.entities.itervalues())
		return "".join(s)

	def apply_serialized_state(self, desc):
		# XXX: Security implications of this line?
		# It's not like we have Javascript-style __proto__ issues, but...
		self.total_game_time = (1.0 - self.SERVER_TIME_ALPHA) * self.total_game_time + self.SERVER_TIME_ALPHA * desc["vars"]["total_game_time"]
		self.next_entity_id = desc["vars"]["next_entity_id"]
		# Unpack server-side configuration.
		self.difficulty = desc["vars"]["difficulty"]
		self.debug = desc["vars"]["debug"]
		# Mark each entity for culling.
		for entity in self.entities.itervalues():
			# By default we cull all entities that aren't client side only.
			if not entity.client_side_only_entity:
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
		applied_ents = []
		while i < len(payload):
			ent_id, = struct.unpack("<i", payload[i:i+4])
			i += 4
			# We now check for special flags in the entity ID.
			# If negative, we delete the given entity.
			if ent_id < 0:
				print "Got kill signal:", ent_id
				if -ent_id in self.entities:
					self.entities.pop(-ent_id)
				continue
			# If the entity ID is over one billion then it's a creation signal.
			if ent_id > ENTITY_CREATION_OFFSET:
				ent_id -= ENTITY_CREATION_OFFSET
				serialization_key = payload[i]
				json_payload_length, = struct.unpack("<I", payload[i+1:i+5])
				json_payload = json.loads(payload[i + 5:i + 5 + json_payload_length])
				i += 5 + json_payload_length
				if ent_id not in self.entities:
					constructor = serialization_key_table[serialization_key]
					self.add_entity(constructor, json_payload, ent_id)
			applied_ents.append(ent_id)
			# If we haven't heard of this object then create it.
			if ent_id not in self.entities:
				print "Invalid ent_id, skipping rest of payload:", ent_id
#				print "Resync will happen at next large update."
				return
#				print "Making a guy:", ent_id, ent_type
#				self.entities[ent_id] = serialization_key_table[ent_type]()
			# Apply our patch, and update the counter.
			i = self.entities[ent_id].apply_patch(payload, i, jitter)
		global STATIC_COUNTER
		STATIC_COUNTER += 1
		if STATIC_COUNTER % 10 == 0 and False:
			print "Entities applied to:", applied_ents

STATIC_COUNTER = 0

class Entity:
	# Set this as a backup so that objects created as children of an object in an initialization routine don't get culled.
	do_cull = False
	client_side_only_entity = False

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
	damage_state = 0

	def deal_damage(self, amount):
		self.hp -= amount
		self.damage_state = 24

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

class MapLoader(Entity):
	serialization_key = "M"

	def init(self, xyz, segment_type):
		self.xyz = xyz
		self.model = render.get_model(segment_type)

		# Extract Bezier curves from the renderer model.
		self.curves = {}
		for curve in self.model.w.metadata["curves"]:
			self.curves[curve["name"]] = compgeom.Bezier(curve["params"])

		# If we're client side we don't try to guess the various objects to be spawned,
		# but instead just wait for the state update to give them to us. So return here.
		if not self.sim.is_server_side:
			return

		for i in xrange(len(self.model.sub_objects)):
			self.sim.add_entity(TerrainSegment, [xyz, segment_type, i])

		# Spawn appropriate entities based on the map's metadata.
		for entity_desc in self.model.w.metadata["entities"]:
			print entity_desc
			thing = entity_desc["type"].lower()
			xyz = entity_desc["xyz"]
			if thing == "spawner":
				# Extract the interval, defaulting to ten seconds.
				print "SPAWNER!"
				interval = entity_desc["properties"].get("interval", 10.0)
				# Look for a key of the form spawn_j, which would be a Jumper spawner.
				spawned_serialization_key = [k for k in entity_desc["properties"] if k.startswith("spawn_")][0][-1]
				self.sim.add_entity(EnemySpawner, [xyz, spawned_serialization_key, interval])
			elif thing.startswith("ser_") and len(thing) == 5:
				# This is a default construction to allow level designers to make objects of any type
				# specifying just serialization key and object coordinates, so we don't need lots of cases.
				constructor = serialization_key_table[thing[-1]]
				self.sim.add_entity(constructor, [xyz])
			else:
				raise ValueError("Unknown entity type: %r" % (thing,))

	def get_xyz(self):
		return self.xyz

	def draw(self):
#		link.glPushMatrix()
#		self.geom.convertIntoReferenceFrame()
		# XXX: TODO: Actually take self.xyz into account! 
		link.set_color(1, 1, 1, 1)
		self.model.render()
#		link.glPopMatrix()

class TerrainSegment(Entity):
	serialization_key = "t"

	def init(self, xyz, segment_type, object_index):
		self.model = render.get_model(segment_type)
		self.object_index = object_index
		self.geom = link.BvhTriangleMesh(self.sim.physics, "stone", self.model.sub_objects[object_index].triangles, self.model.sub_objects[object_index].triangles_texture_coords, xyz, (1, 0, 0, 0))

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
		# XXX: FIXME: For some reason if I set both position AND angle then the object
		# loses all locally predicted motion, and is basically a streamed animation... :(
		self.geom.setPos(pos)
#		self.geom.setAxisAngle(axis_angle)
		self.geom.setLinearVelocity(velo)
		return i + 40

	def draw(self):
		obj = self.geom
		x, y, z = obj.getPos()
		t1, t2, t3 = obj.bounds[0]/2.0, obj.bounds[1]/2.0, obj.bounds[2]/2.0
		link.glPushMatrix()
		obj.convertIntoReferenceFrame()
		bounds = [-t1, +t1, -t2, +t2, -t3, +t3]
		link.draw_box(bounds, "data/crate.png")
		link.glPopMatrix()

class Boulder(Block):
	serialization_key = "B"
	mass = 120.0
	radius = 4.0

	def collision(self, other, dt):
		# Detect collisions with the player and deal damage, but only server-side.
		if isinstance(other, Player):
			other.disable_movement_control()
			delta_pos = other.get_xyz() - self.get_xyz()
			delta_pos /= max(0.01, np.linalg.norm(delta_pos))
			delta_pos *= 10.0 * dt
#			other.geom.applyForce(delta_pos[0], delta_pos[1], delta_pos[2])
			velo = other.geom.getLinearVelocity()
			our_velo = self.geom.getLinearVelocity()
			delta = np.array(our_velo) - np.array(velo)
			delta *= dt
			delta *= 1000.0
#			other.geom.applyForce(delta[0], delta[1], delta[2])
			other.geom.setLinearVelocity(our_velo)

	def init(self, xyz):
#		self.model = render.get_model("block")
		self.geom = link.Sphere(self.sim.physics, "stone", self.radius, xyz, (1, 0, 0, 0), self.mass)
		self.duration = 10.0

	def step(self, dt):
		self.duration -= dt
		if self.duration <= 0.0 and self.sim.is_server_side:
			self.destroy()

	def draw(self):
		obj = self.geom
		x, y, z = obj.getPos()
		#t1, t2, t3 = obj.bounds[0]/2.0, obj.bounds[1]/2.0, obj.bounds[2]/2.0
		t1, t2, t3 = self.radius, self.radius, self.radius
		link.glPushMatrix()
		obj.convertIntoReferenceFrame()
#		bounds = [-t1, +t1, -t2, +t2, -t3, +t3]
#		link.draw_box(bounds, "data/crate.png")
		link.set_color(1, 1, 1, 1)
		render.get_model("boulder").render()
		link.glPopMatrix()

class Jumper(Entity, GeomOnlyPatchMixin, EnemyMixin):
	serialization_key = "j"
	max_aggro_range = 50.0
	radius = 1.4
	gl_scale = 1.2
	max_hp = 20
	damage = 40
	mass = 2.0

	leap_coef = 0.5
	max_leap_velocity = 20.0
	leap_vertical_component = 10

	def init(self, xyz):
		self.geom = link.Sphere(self.sim.physics, "stone", self.radius, xyz, (1, 0, 0, 0), self.mass, collision_group=COLLISION_ENEMY)
		self.aggro_target = -1
		self.jump_cooldown = 0.0
		self.hp = self.max_hp
		self.lifespan = 10.0

	def serialize_patch(self):
		base_patch = GeomOnlyPatchMixin.serialize_patch(self)
		base_patch += struct.pack("<bfi", self.damage_state, self.jump_cooldown, self.aggro_target)
		return base_patch

	def apply_patch(self, patch, i, jitter=0.0):
		i = GeomOnlyPatchMixin.apply_patch(self, patch, i, jitter)
		self.damage_state, self.jump_cooldown, self.aggro_target = struct.unpack("<bfi", patch[i:i+9])
		return i + 9

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

		# TODO: Think carefully about framerate independence here.
		if self.damage_state > 0:
			self.damage_state -= 1

		# Server side maximum lifespan.
		self.lifespan -= dt
		if (self.lifespan <= 0.0 or self.hp <= 0) and self.sim.is_server_side:
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
		if self.damage_state % 12 >= 6:
			link.set_color(0, 0, 0, 1)
		else:
			link.set_color(1, 1, 1, 1)
		render.get_model("jumper").render()
		link.glPopMatrix()

	def collision(self, other, dt):
		# Detect collisions with the player and deal damage, but only server-side.
		if isinstance(other, Player) and self.sim.is_server_side:
			print "Hit the player!"
			self.destroy()
			other.deal_damage(self.damage)

class BigJumper(Jumper):
	serialization_key = "J"
	max_aggro_range = 50.0
	radius = 1.4 * 1.6
	gl_scale = 1.2 * 1.6
	max_hp = 50
	damage = 55
	mass = 8.0

	leap_coef = 0.9
	max_leap_velocity = 20.0
	leap_vertical_component = 8

class EnemySpawner(Entity):
	serialization_key = "s"
	max_aggro_range = 80.0

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
			# Only spawn enemies on the server side.
			if self.sim.is_server_side:
				if self.sim.difficulty > 0:
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
	cart_speed = 1.5
	contest_distance = 8.0

	def init(self, xyz):
		self.geom = link.Box(self.sim.physics, "stone", (2.0, 4.0, 1.5), xyz, (0, 0, -1, math.pi/2), self.mass)
		self.geom.setAngularFactor(0.0, 0.0, 0.0)
		self.geom.setGravity(0.0, 0.0, 0.0)
		self.geom.setKinematic(True)
		self.push_state = "idle"
		self.accumulated_movement = 0.0
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
		link.set_color(1, 1, 1, 1)
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
		if self.sim.debug:
			multiplier *= 10.0
		self.current_speed = multiplier * self.cart_speed
		self.accumulated_movement += self.current_speed * dt
		#self.geom.setLinearVelocity((multiplier * self.cart_speed, 0, 0))
		level_gen = self.sim.get_ent(lambda x: isinstance(x, RandomlyGeneratedTerrain))
		if level_gen != None:
			level_gen = level_gen.lg
			new_xyz, new_theta = level_gen.get_xyz_theta_on_path(self.accumulated_movement)
		else:
			# Otherwise, look for a terrain segment with a Bezier curve.
			level_gen = self.sim.get_ent(lambda x: isinstance(x, MapLoader))
			curve = next(level_gen.curves.itervalues())
			new_xyz = curve.get_point_by_distance(self.accumulated_movement)
			new_deriv = curve.get_derivative_by_distance(self.accumulated_movement)
			new_theta = math.atan2(new_deriv[1], new_deriv[0])
		new_xyz += np.array([0, 0, 1.0])
		self.current_theta = 0.95 * self.current_theta + 0.05 * new_theta
		self.geom.setPos(new_xyz)
		self.geom.setAxisAngle((0, 0, -1, math.pi/2 - self.current_theta))

class Effect:
	def step(self, dt):
		pass

	def draw(self):
		pass

	def destroy(self):
		self.sim.remove_effect(self)

class Explosion(Effect):
	def init(self, xyz):
		self.xyz = np.array(xyz)
		self.duration = 0.5

	def step(self, dt):
		self.duration -= dt
		if self.duration <= 0.0:
			self.destroy()

	def draw(self):
		lower = - np.array([1, 1, 1]) * self.duration * 2.0
		upper = + np.array([1, 1, 1]) * self.duration * 2.0
		link.glPushMatrix()
		link.glTranslatef(*self.xyz)
		link.draw_box([lower[0], upper[0], lower[1], upper[1], lower[2], upper[2]], "data/lava_n4pgamer.png")
		link.glPopMatrix()

class Bullet(Effect):
	velocity = 100.0
	total_range = 200.0
	client_side_only_entity = True

	def init(self, xyz, facing, tilt):
		self.xyz, self.facing, self.tilt = np.array(xyz), facing, tilt
#		print "Heading towards:", self.facing, self.tilt
		self.direction = np.array([
			math.sin(facing) * math.cos(tilt),
			math.cos(facing) * math.cos(tilt),
			-math.sin(tilt),
		])
		self.total_movement = 0.0
		self.beginning_of_step_position = self.xyz

	def get_xyz(self):
		return self.xyz

	def end_bullet_path(self):
		self.sim.add_effect(Explosion, [self.xyz])
		self.destroy()

	def step(self, dt):
		self.beginning_of_step_position = self.xyz.copy()
		# Compute a maximum distance we'd like to move in this frame.
		max_movement_distance = min(self.total_range - self.total_movement, self.velocity * dt)
		target_point = self.xyz + max_movement_distance * self.direction
		# Cast a ray towards the target destination.
		hit = self.sim.physics.rayCast(self.xyz, target_point, COLLISION_SOLID | COLLISION_ENEMY)
		if not hit:
			# On no hit we move the full distance.
			movement = max_movement_distance
		else:
			# If we did hit, then determine if the 
			distance_to_hit = compgeom.l2(self.xyz, hit.xyz)
			if distance_to_hit > max_movement_distance:
				movement = max_movement_distance
			else:
				# We hit something close!
				hit_entity = self.sim.get_ent(lambda e: getattr(e, "geom", None) == hit.hit_object)
				if isinstance(hit_entity, EnemyMixin):
					if self.sim.is_server_side:
						hit_entity.deal_damage(12)
				self.xyz = hit.xyz
				self.end_bullet_path()
				return
		self.xyz += movement * self.direction
		self.total_movement += movement
		if self.total_range - self.total_movement < 1e-3:
			self.end_bullet_path()

	def draw(self):
		xyz = self.beginning_of_step_position.copy()
		while compgeom.l2(xyz, self.xyz) > 0.5:
			lower = - np.array([1, 1, 1]) * 0.1
			upper = + np.array([1, 1, 1]) * 0.1
			link.glPushMatrix()
			link.glTranslatef(*xyz)
			link.draw_box([lower[0], upper[0], lower[1], upper[1], lower[2], upper[2]], "data/lava_n4pgamer.png")
			link.glPopMatrix()
			xyz += 0.5 * self.direction

class RespawnHandle(Entity):
	serialization_key = "R"
	respawn_time = 6.0

	def init(self, name, player_ent_id):
#		print "CREATING RESPAWN HANDLE:", name, player_ent_id
		self.name, self.player_ent_id = name, player_ent_id
		self.respawn_cooldown = self.respawn_time

	def get_xyz(self):
		return np.array([-100, -100, -100])

	def step(self, dt):
		self.respawn_cooldown -= dt
		# Only do server-side respawn computations.
		if self.respawn_cooldown <= 0.0 and self.sim.is_server_side:
			print "Respawning player: %r (%i)" % (self.name, self.player_ent_id)
			player = self.sim.add_entity(Player, [(0, 0, 0), self.name], specific_ent_id=self.player_ent_id)
			player.respawn()
			self.destroy()

class Player(GeomOnlyPatchMixin, Entity):
	serialization_key = "p"
	move_speed = 6.0
	move_stiffness = 15.0 * 60.0
	jump_velocity = 8.0
	jump_count = 1
	mass = 1.0
	radius = 0.5
	eye_height = 0.75
	is_actively_controlled_player = False

	max_ammo = 8
	shoot_interval = 0.4
	reload_interval = 1.5
	max_hp = 100.0
	health_regen_interval = 3.0
	health_regen_rate = 2.0

	def init(self, xyz, name):
		self.name = name
		self.geom = link.Sphere(self.sim.physics, "stone", self.radius, xyz, (1, 0, 0, 0), self.mass, collision_group=COLLISION_PLAYER)
		self.geom.setAngularFactor(0.0, 0.0, 0.0)
		self.input_motion_vector = (0.0, 0.0)
		self.facing = 0.0
		self.tilt = 0.0
		self.jumps_available = 0
		self.hp = self.max_hp
		self.ammo = self.max_ammo
		self.ammo_claim_patches = 0
		self.shoot_cooldown = 0.0
		self.reload_cooldown = 0.0
		self.health_regen_cooldown = 0.0
		self.movement_cooldown = 0.0
#		self.player_class_state = characters.BasicCharacter()

#	def init_from_state(self, desc):
#		self.name = desc["name"]

#	def serialize_state(self):
#		return {"name": self.name}

	def serialize_patch(self):
		s = []
		s.append(struct.pack("<3f", *self.geom.getPos()))
		s.append(struct.pack("<3f", *self.geom.getLinearVelocity()))
		s.append(struct.pack("<2f", *self.input_motion_vector))
		s.append(struct.pack("<hfh", self.jumps_available, self.hp, self.ammo))
		return "".join(s)

	def apply_patch(self, patch, i, jitter=0.0):
		pos  = struct.unpack("<3f", patch[i:i+12])
		velo = struct.unpack("<3f", patch[i+12:i+24])
		motion_vector = struct.unpack("<2f", patch[i+24:i+32])
		old_hp = self.hp
		self.jumps_available, self.hp, claimed_ammo = struct.unpack("<hfh", patch[i+32:i+40])
		# If our HP was lowered, then infer that we should set the health_regen_cooldown on the client side.
		if self.hp < old_hp:
			self.health_regen_cooldown = self.health_regen_interval
		if USE_JITTER_CORRECTION:
			pos = np.array(pos) - np.array(velo) * jitter
#		print self, "patch:", pos, velo
		self.geom.setPos(pos)
		self.geom.setLinearVelocity(velo)
		if not self.is_actively_controlled_player:
			self.input_motion_vector = motion_vector
		# We now apply some special-cased smoothing to the claimed ammo, to prevent jitter.
		if claimed_ammo > self.ammo:
			self.ammo_claim_patches += 1
			# We only accept claims of an increase in ammo if they're claimed for at least three consecutive patches.
			# This smoothes out the ammo counter.
			if self.ammo_claim_patches >= 15:
				self.ammo = claimed_ammo
				self.ammo_claim_patches = 0
		else:
			self.ammo = claimed_ammo
			self.ammo_claim_patches = 0
		return i + 4 * 8 + 2 + 4 + 2

	def draw(self):
		if self.is_actively_controlled_player:
			return
		link.glPushMatrix()
		self.geom.convertIntoReferenceFrame()
		link.set_color(1, 1, 1, 1)
		render.get_model("player_model").render()
		link.glPopMatrix()

	def deal_damage(self, amount):
		self.hp -= amount
		self.health_regen_cooldown = self.health_regen_interval

	def disable_movement_control(self):
		self.movement_cooldown = 0.3

	def step(self, dt):
		# XXX: Not framerate independent yet!
		self.apply_forces(dt)
		self.elevation = self.get_elevation(collision_mask=COLLISION_SOLID | COLLISION_ENEMY, corrections=self.radius)
		if self.elevation < 0.4 or self.sim.debug:
			self.jumps_available = self.jump_count
		else:
			# Prevent the player from getting two full jumps if they walk off an edge.
			self.jumps_available = min(self.jumps_available, self.jump_count - 1)

		self.shoot_cooldown = max(0.0, self.shoot_cooldown - dt)
		self.reload_cooldown = max(0.0, self.reload_cooldown - dt)
		self.health_regen_cooldown = max(0.0, self.health_regen_cooldown - dt)
		self.movement_cooldown = max(0.0, self.movement_cooldown - dt)
		if self.ammo == 0 and self.reload_cooldown == 0.0:
			self.ammo = self.max_ammo
		if self.hp < self.max_hp and self.health_regen_cooldown == 0.0:
			self.hp = min(self.max_hp, self.hp + dt * self.health_regen_rate)

		# On the server we kill players if they reach zero or less HP.
		# Further, kill the player if they fall below the plane z = -20.
		if (self.hp <= 0.0 or self.get_xyz()[2] < -20.0) and self.sim.is_server_side:
			self.kill_the_player()

	def kill_the_player(self):
		print "Killing player: %r (%i)" % (self.name, self.ent_id)
		self.destroy()
		# Create a respawn handle that will regenerate us.
		self.sim.add_entity(RespawnHandle, [self.name, self.ent_id])

	def respawn(self):
		# Respawn over the cart.
#		cart = self.sim.get_ent(lambda ent: isinstance(ent, Cart))
		map_loader = self.sim.get_ent(lambda x: isinstance(x, MapLoader))
		for location_name, xyz in map_loader.model.w.metadata["marker_locations"].iteritems():
			if location_name == "M_Spawn%i" % self.sim.current_checkpoint_value:
				print "Using location", location_name, "at", xyz
				self.geom.setPos(xyz)
				break
		else:
			print "ERROR! No spawn location for checkpoint value %i." % self.sim.current_checkpoint_value
		self.geom.setLinearVelocity((0, 0, 0))

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

	def try_to_shoot(self):
		if self.ammo > 0 and self.shoot_cooldown == 0.0:
			self.shoot_cooldown = self.shoot_interval
			self.ammo -= 1
			if self.ammo == 0:
				self.reload_cooldown = self.reload_interval
			# Shoot, but only for the client.
			self.sim.add_effect(Bullet, [self.get_xyz(), self.facing, self.tilt])

	def try_to_command(self, command):
		if command == "jump":
			self.try_to_jump()
		elif command == "m1":
			self.try_to_shoot()
		else:
			raise ValueError("Bad command: %r" % (command,))

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
		if self.sim.debug:
			x *= 1.5
			y *= 1.5
		# WARNING: The value 10000 is the maximum height we can be over terrain before the ray is missed!
		MAX_ELEVATION = 10000
		dx, dy = self.move_speed*x - vel[0], self.move_speed*y - vel[1]
		if self.movement_cooldown == 0.0:
			self.geom.applyForce(self.move_stiffness * dx * dt, self.move_stiffness * dy * dt, 0)

serialization_key_table = {}
for cls in globals().values():
	if hasattr(cls, "serialization_key"):
		assert cls.serialization_key not in serialization_key_table, "Collision in serialization_keys!"
		serialization_key_table[cls.serialization_key] = cls

def initialize_game(sim):
	# Create a cart at the origin.
	sim.add_entity(Cart, [(0, 0, 1.0)])
#	sim.add_entity(EnemySpawner, [(20, 0, 2), "j", 5.0])
#	sim.add_entity(BigJumper, [(21, 1, 10)])
#	for i in xrange(50):
#		sim.add_entity(Block, [(20.0 + i * 0.05, i * 0.05, 1.0 + i * 2.0)])

	sim.add_entity(MapLoader, [(0, 0, 0), "map1"])
#	sim.add_entity(RandomlyGeneratedTerrain, [(0, 0, 0), 10])

