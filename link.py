#! /usr/bin/python
"""
link -- Interface to the C++ shared object.
"""

import ctypes, os, sys, platform, fractions, time
from ctypes import c_int, c_longlong, c_float, c_double, c_char_p, POINTER
import compgeom

Real = c_double

search_path = ["", "./", ".\\", os.path.dirname(__file__) + "/", "/usr/lib/", "/usr/local/lib/"]

loaded = []
def load(func, suffix, optional=False):
	for path in search_path:
		path += suffix
		try:
			result = func(path)
			# Store a reference.
			loaded.append(result)
			return result
		except OSError as e:
			pass
#			print "Couldn't load:", path
#			print e
	if not optional:
		raise Exception("Couldn't load %s with any prefix." % suffix)

if "windows" in platform.system().lower():
	# First load all the deps.
	load(ctypes.WinDLL, "zlib1.dll", optional=True)
	load(ctypes.WinDLL, "libpng12-0.dll", optional=True)
	load(ctypes.WinDLL, "SDL.dll", optional=True)
	opengl_library = load(ctypes.WinDLL, "opengl32.dll")
	library = load(ctypes.CDLL, "libcartpusher.dll")
else:
	load(ctypes.CDLL, "libLinearMath.so.2.83", optional=True)
	load(ctypes.CDLL, "libBulletCollision.so.2.83", optional=True)
	load(ctypes.CDLL, "libBulletDynamics.so.2.83", optional=True)
	library = load(ctypes.CDLL, "libcartpusher.so")
	opengl_library = library

def get(name, restype=None, argtypes=[], masked=False, library=library):
	f = getattr(library, name) 
	f.restype = restype
	f.argtypes = argtypes
	globals()["_"[:masked] + name] = f

def address_from_pointer(pointer):
	# I don't know a better ctypes idiom to handle null pointers.
	if not pointer:
		return 0
	return ctypes.addressof(pointer.contents)

class NoTexture:
	index = -1

texture_map = {None: NoTexture}

class Texture:
	def __init__(self, texture_name):
		buf = (ctypes.c_int * 2)()
		# Normalize so / is the separator on all systems.
#		texture_name = os.path.join(*texture_name.split("/"))
#		texture_name = os.path.join("data", texture_name + ".png")
		self.index = png_texture_load(texture_name, buf, False)
		self.width, self.height = buf[0], buf[1]

def get_texture(texture_name):
	# If the texture name is unheard of, load it up.
	if texture_name not in texture_map:
		texture_map[texture_name] = Texture(texture_name)
	return texture_map[texture_name]

class Event(ctypes.Structure):
	_fields_ = [("type", c_int),
	            ("key", c_int),
	            ("button", c_int)]

class PhysicsObject(ctypes.Structure):
	pass

class PhysicsWorld(ctypes.Structure):
	pass

PhysicsObjectPointer = POINTER(PhysicsObject)
PhysicsWorldPointer = POINTER(PhysicsWorld)

E_QUIT = 1
E_MOUSE_DOWN = 2
E_MOUSE_UP = 3
E_KEY_DOWN = 4
E_KEY_UP = 5
K_UP = 273
K_DOWN = 274
K_LEFT = 276
K_RIGHT = 275
K_L_SHIFT = 304
K_R_SHIFT = 303
K_L_CTRL = 306
K_R_CTRL = 305
K_L_ALT = 308
K_R_ALT = 307
K_L_META = 310
K_R_META = 309
K_L_SUPER = 312
K_R_SUPER = 311
K_SPACE = 32
K_RETURN = 13
K_ESCAPE = 27
K_TAB = 9
K_BACKSPACE = 8

# Load up the library global variables.
for c_type, names in [
	(c_int, ["screen_width", "screen_height", "mouse_x", "mouse_y"]),
	(c_float, ["camera_x", "camera_y", "camera_z"]),
	(c_float, ["camera_tilt", "camera_facing"]),
	(Event, ["last_event"])]:
	for name in names:
		globals()[name] = c_type.in_dll(library, name)

# Load up the main library methods.
get("graphics_init", c_int, [c_int])
get("graphics_close")
get("png_texture_load", c_int, [c_char_p, POINTER(c_int), c_int])
get("begin_frame")
get("begin_overlay")
get("end_frame")
get("get_text_width", c_float, [c_char_p, c_float])
get("draw_with_font", None, [c_char_p, c_float, c_float, c_float])
get("set_color", None, [c_float, c_float, c_float, c_float])
get("translate", None, [c_float, c_float, c_float])
get("draw_triangle", None, [POINTER(c_float), c_int], masked=True)
get("draw_sphere", None, [c_float, c_float, c_float, c_float])
get("draw_triangles", None, [c_int, POINTER(c_float), POINTER(c_float), c_int], masked=True)
get("draw_box", None, [POINTER(c_float), c_int], masked=True)
get("get_event", c_int)
get("warp_mouse", None, [c_int, c_int])
get("gettimeofday_wrapper", None, [POINTER(c_longlong)], masked=True)
#get("play_music", None, [c_char_p])
#get("play_sound", None, [c_char_p])

# Simple GL wrappers.
get("glPushMatrix", None, [], library=opengl_library)
get("glPopMatrix", None, [], library=opengl_library)
get("glScalef", None, [c_float, c_float, c_float], library=opengl_library)
get("glTranslatef", None, [c_float, c_float, c_float], library=opengl_library)

GL_COMPILE = 0x1300
# Technically all these c_ints are actually GLsizei, but it's typedefed to int in gl.h, so I think this is safe...
get("glGenLists", c_int, [c_int], library=opengl_library)
get("glNewList", None, [c_int, c_int], library=opengl_library)
get("glEndList", None, [], library=opengl_library)
get("glCallList", None, [c_int], library=opengl_library)
get("glDeleteLists", None, [c_int, c_int], library=opengl_library)

# Physics.
get("PhysicsWorld_new", PhysicsWorldPointer, [])
get("PhysicsWorld_step", None, [PhysicsWorldPointer, Real])
get("PhysicsWorld_rayCast", c_int, [PhysicsWorldPointer, POINTER(Real), POINTER(Real), POINTER(Real), POINTER(PhysicsObjectPointer), c_int])
get("PhysicsWorld_checkForContact", c_int, [PhysicsWorldPointer, PhysicsObjectPointer, PhysicsObjectPointer, c_int])
get("PhysicsWorld_listAllCollidingPairs", c_int, [PhysicsWorldPointer, POINTER(POINTER(PhysicsObjectPointer))])
get("delete_colliding_pairs_list", None, [POINTER(PhysicsObjectPointer)])

get("Box_new", PhysicsObjectPointer, [PhysicsWorldPointer, POINTER(Real), POINTER(Real), POINTER(Real), Real, c_int])
get("Sphere_new", PhysicsObjectPointer, [PhysicsWorldPointer, Real, POINTER(Real), POINTER(Real), Real, c_int])
get("ConvexHull_new", PhysicsObjectPointer, [PhysicsWorldPointer, POINTER(Real), POINTER(Real), c_int, POINTER(Real), Real, c_int])
get("Capsule_new", PhysicsObjectPointer, [PhysicsWorldPointer, Real, Real, POINTER(Real), POINTER(Real), Real, c_int])
get("PlayerShape_new", PhysicsObjectPointer, [PhysicsWorldPointer, Real, Real, Real, POINTER(Real), POINTER(Real), Real, c_int])
get("BvhTriangleMesh_new", PhysicsObjectPointer, [PhysicsWorldPointer, c_int, POINTER(Real), POINTER(Real), POINTER(Real), c_int])

get("PhysicsObject_removeFromWorld", None, [PhysicsObjectPointer])
get("PhysicsObject_getPos", None, [PhysicsObjectPointer, POINTER(Real)])
get("PhysicsObject_setPos", None, [PhysicsObjectPointer, Real, Real, Real])
get("PhysicsObject_getAxisAngle", None, [PhysicsObjectPointer, POINTER(Real)])
get("PhysicsObject_setAxisAngle", None, [PhysicsObjectPointer, Real, Real, Real, Real])
get("PhysicsObject_setPosAxisAngle", None, [PhysicsObjectPointer, Real, Real, Real, Real, Real, Real, Real])
get("PhysicsObject_getLinearVelocity", None, [PhysicsObjectPointer, POINTER(Real)])
get("PhysicsObject_setLinearVelocity", None, [PhysicsObjectPointer, Real, Real, Real])
get("PhysicsObject_convertIntoReferenceFrame", None, [PhysicsObjectPointer])
get("PhysicsObject_applyForce", None, [PhysicsObjectPointer, Real, Real, Real])
get("PhysicsObject_applyCentralImpulse", None, [PhysicsObjectPointer, Real, Real, Real])
get("PhysicsObject_setAngularFactor", None, [PhysicsObjectPointer, Real, Real, Real])
get("PhysicsObject_setGravity", None, [PhysicsObjectPointer, Real, Real, Real])
get("PhysicsObject_setKinematic", None, [PhysicsObjectPointer, c_int])

#get("create_block", PhysicalObjectPointer, [POINTER(c_float), c_int, POINTER(c_float)], masked=True)
#get("draw_block", None, [POINTER(c_float), c_int, POINTER(c_float)], masked=True)
#get("ObjectSet_create", ObjectSetPointer, [])
#get("ObjectSet_append_object", None, [ObjectSetPointer, PhysicalObjectPointer])
#get("ObjectSet_bounds_check", c_int, [ObjectSetPointer, c_float, c_float, c_float])
#get("ObjectSet_push_onto_surface", None, [ObjectSetPointer, POINTER(c_float)])
#get("ObjectSet_recenter_bounding_box", None, [ObjectSetPointer, POINTER(c_float), c_float, c_float, c_float])
#get("ObjectSet_draw_self", None, [ObjectSetPointer])

# Convenience routines.

def draw_bordered_text(xy, text, font_size=30, color=(1, 1, 1, 1)):
	set_color(0, 0, 0, 1)
	# Make a dark outline.
	draw_with_font(text, xy[0]-1, xy[1]-1, font_size)
	draw_with_font(text, xy[0]+1, xy[1]+1, font_size)
	set_color(*color)
	draw_with_font(text, xy[0], xy[1], font_size)
	return get_text_width(text, font_size)

def draw_centered_text(text, font_size=50):
	width = get_text_width(text, font_size)
	x = screen_width.value / 2 - width / 2
	y = screen_height.value / 2 - font_size / 2
	set_color(0, 0, 0, 1)
	# Make a dark outline.
	draw_with_font(text, x-1, y-1, font_size)
	draw_with_font(text, x+1, y+1, font_size)
	set_color(1, 1, 1, 1)
	draw_with_font(text, x, y, font_size)

# Masked routines.

def draw_triangle(coords, texture=-1):
	array = (ctypes.c_float*9)(*coords[0]+coords[1]+coords[2])
	_draw_triangle(array, texture)

def pack_triangles_into_array(triangles, t=ctypes.c_float, coords_per_vertex=3):
	coords = []
	for triangle in triangles:
		coords.extend(list(triangle[0] + triangle[1] + triangle[2]))
	array = (t*len(coords))()
	for i, v in enumerate(coords):
		array[i] = v
	assert len(triangles)*(3 * coords_per_vertex) == len(coords)
	return array

def pack_triangles(triangles, texture_coords):
	array1 = pack_triangles_into_array(triangles)
	array2 = pack_triangles_into_array(texture_coords, coords_per_vertex=2)
	return array1, array2

def draw_triangles(arrays, texture=-1):
	_draw_triangles(len(arrays[0])/9, arrays[0], arrays[1], texture)

def create_block(rgba, texture_name, bounds):
	array1 = (ctypes.c_float*4)(*rgba)
	array2 = (ctypes.c_float*6)(*bounds)
	return _create_block(array1, get_texture(texture_name).index, array2)

def draw_box(bounds, texture_name):
	#array1 = (ctypes.c_float*4)(*rgba)
	array2 = (ctypes.c_float*6)(*bounds)
	return _draw_box(array2, get_texture(texture_name).index)

def gettimeofday_wrapper():
	return time.time()
#	result = (ctypes.c_longlong*2)()
#	_gettimeofday_wrapper(result)
#	seconds, microseconds = result[0], result[1]
#	return seconds + fractions.Fraction(microseconds, 1000000)

# Classes to wrap the C++ classes.

class PhysicsObject:
	visible = True

	def __init__(self, parent):
		addr = address_from_pointer(self.ptr)
		parent.pointer_to_physics_object[addr] = self

	def removeFromWorld(self):
		if self.ptr != None:
			PhysicsObject_removeFromWorld(self.ptr)

	def getPos(self):
		array = (Real*3)()
		PhysicsObject_getPos(self.ptr, array)
		return array[:3]

	def setPos(self, xyz):
		x, y, z = xyz
		PhysicsObject_setPos(self.ptr, x, y, z)

	def getAxisAngle(self):
		array = (Real*4)()
		PhysicsObject_getAxisAngle(self.ptr, array)
		return array[:4]

	def setAxisAngle(self, axis_angle):
		x, y, z, t = axis_angle
		PhysicsObject_setAxisAngle(self.ptr, x, y, z, t)

	def setPosAxisAngle(self, pos, axis_angle):
		xx, yy, zz = pos
		x, y, z, t = axis_angle
		PhysicsObject_setPosAxisAngle(self.ptr, xx, yy, zz, x, y, z, t)

	def getLinearVelocity(self):
		array = (Real*3)()
		PhysicsObject_getLinearVelocity(self.ptr, array)
		return array[:3]

	def setLinearVelocity(self, velocity):
		PhysicsObject_setLinearVelocity(self.ptr, velocity[0], velocity[1], velocity[2])

	def convertIntoReferenceFrame(self):
		PhysicsObject_convertIntoReferenceFrame(self.ptr)

	def applyForce(self, x, y, z):
		PhysicsObject_applyForce(self.ptr, x, y, z)

	def applyCentralImpulse(self, x, y, z):
		PhysicsObject_applyCentralImpulse(self.ptr, x, y, z)

	def setAngularFactor(self, x, y, z):
		PhysicsObject_setAngularFactor(self.ptr, x, y, z)

	def setGravity(self, x, y, z):
		PhysicsObject_setGravity(self.ptr, x, y, z)

	def setKinematic(self, is_kinematic):
		PhysicsObject_setKinematic(self.ptr, int(bool(is_kinematic)))

class Box(PhysicsObject):
	def __init__(self, parent, texture, bounds, xyz, rotation, mass, collision_group=1):
		self.bounds = bounds
		self.texture = texture
		bounds = (Real*3)(*bounds)
		xyz = (Real*3)(*xyz)
		rotation = (Real*4)(*rotation)
		self.ptr = Box_new(parent.world_ptr, bounds, xyz, rotation, mass, collision_group)
		PhysicsObject.__init__(self, parent)

class Sphere(PhysicsObject):
	def __init__(self, parent, texture, radius, xyz, rotation, mass, collision_group=1):
		self.radius = radius
		self.texture = texture
		xyz = (Real*3)(*xyz)
		rotation = (Real*4)(*rotation)
		self.ptr = Sphere_new(parent.world_ptr, radius, xyz, rotation, mass, collision_group)
		PhysicsObject.__init__(self, parent)

class ConvexHull(PhysicsObject):
	def __init__(self, parent, texture, xyz, rotation, points, mass, collision_group=1):
		self.texture = texture
		xyz = (Real*3)(*xyz)
		rotation = (Real*4)(*rotation)
		points_array = (Real*(3*len(points)))()
		for i, point in enumerate(points):
			assert len(point) == 3, "ConvexHull was fed point with %i coords, not three." % len(point)
			for j, coord in enumerate(point):
				points_array[i*3 + j] = coord
		self.ptr = ConvexHull_new(parent.world_ptr, xyz, rotation, len(points), points_array, mass, collision_group)
		PhysicsObject.__init__(self, parent)

class Capsule(PhysicsObject):
	def __init__(self, parent, texture, radius, height, xyz, rotation, mass, collision_group=1):
		self.radius = radius
		self.texture = texture
		xyz = (Real*3)(*xyz)
		rotation = (Real*4)(*rotation)
		self.ptr = Capsule_new(parent.world_ptr, radius, height, xyz, rotation, mass, collision_group)
		PhysicsObject.__init__(self, parent)

class PlayerShape(PhysicsObject):
	def __init__(self, parent, texture, radius, cone_height, torso_height, xyz, rotation, mass, collision_group=1):
		self.radius = radius
		self.texture = texture
		xyz = (Real*3)(*xyz)
		rotation = (Real*4)(*rotation)
		self.ptr = PlayerShape_new(parent.world_ptr, radius, cone_height, torso_height, xyz, rotation, mass, collision_group)
		PhysicsObject.__init__(self, parent)

class BvhTriangleMesh(PhysicsObject):
	def __init__(self, parent, texture, triangles, texture_coords, xyz, rotation, collision_group=1):
		self.texture = texture
		self.triangles = triangles
		# We compute transformed triangles, in global coordinates. This is currently used just for the add-on.
		# TODO: Make this not use Rodrigues' formula for each triangle, but instead compute a rotation matrix.
		# Even better, use numpy broadcasting to efficiently apply the rotation matrix to every triangle at once.
		# Also, only do it if we're running as part of the add-on, rather than the main game.
		repacked_rotation = rotation[:3], rotation[3]
		def transform_triangle(triangle):
			return [compgeom.add(xyz, compgeom.apply_axis_angle_rotation_to_vector(repacked_rotation, point)) for point in triangle]
		self.transformed_triangles = list(map(transform_triangle, triangles))
		self.texture_coords = texture_coords
		self.array = pack_triangles_into_array(triangles, t=Real)
		self.packed_triangles = pack_triangles(self.triangles, self.texture_coords)
		xyz = (Real*3)(*xyz)
		rotation = (Real*4)(*rotation)
#		if actually_have_physics:
		self.ptr = BvhTriangleMesh_new(parent.world_ptr, len(self.triangles), self.array, xyz, rotation, collision_group)
#		else:
#			self.ptr = None
		PhysicsObject.__init__(self, parent)

class RayHit:
	def __init__(self, hit_object, xyz):
		self.hit_object, self.xyz = hit_object, xyz

class PhysicsWorld:
	def __init__(self):
		self.world_ptr = PhysicsWorld_new()
		self.pointer_to_physics_object = {}

	def step(self, dt):
		PhysicsWorld_step(self.world_ptr, dt)

	def rayCast(self, start, end, collision_mask=0x7fff):
		a1 = (Real*3)(*start)
		a2 = (Real*3)(*end)
		res = (Real*3)()
		hit_object = PhysicsObjectPointer()
		if PhysicsWorld_rayCast(self.world_ptr, a1, a2, res, ctypes.byref(hit_object), collision_mask):
#			print hit_object.contents
			hit_address = address_from_pointer(hit_object)
			hit_object = self.pointer_to_physics_object[hit_address]
			# Figure out who this is.
			return RayHit(hit_object, res[:3])
		# Return None if there is no hit.

	def checkForContact(self, objA, objB, collision_mask=0x7fff):
		assert False, "This routine currently doesn't work."
		return PhysicsWorld_checkForContact(objA.ptr, objB.ptr, collision_mask)

	def listAllCollidingPairs(self):
		ptr = POINTER(PhysicsObjectPointer)()
		count = PhysicsWorld_listAllCollidingPairs(self.world_ptr, ctypes.byref(ptr))
		# Pull out the packed array of pairs of PhysicsObject pointers.
		# Every two entries is a packed pair.
		results = ptr[:count * 2]
		# Look up the physics objects by the pointers they contain.
		results = [self.pointer_to_physics_object[address_from_pointer(i)] for i in results]
		# Pack the flattened list into tuple pairs.
		pairs = list(zip(*[iter(results)]*2))
		# We are responsible for freeing the structure allocated by PhysicsWorld_listAllCollidingPairs().
		delete_colliding_pairs_list(ptr)
		return pairs

if __name__ == "__main__":
	w = PhysicsWorld()
	floor = Box(w, None, (10, 10, 1), (0, 0, 0), (1, 0, 0, 0), 0)
	ball = Sphere(w, None, 1, (0, 0, 8), (1, 0, 0, 0), 1)
	for _ in range(20):
		print(ball.getPos(), w.listAllCollidingPairs())
		w.step(0.1)

