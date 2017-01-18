#!/usr/bin/python

from __future__ import print_function

import os, math, json, itertools
import compgeom

# Now that I am exporting objs with axis_forward="Y", axis_up="Z" I no longer need to swap axes.
# But, before I did, so I have changed this mapping to the identity.
#sf = __import__("GLOBALS").scale_factor
sf = 1.0
AXIS_SWAP = lambda x: (sf*x[0], sf*x[1], sf*x[2])

def extract_name(n):
	# Because Blender appends stuff, we cut down the name to the first underscore separated component, with the possible exception of some prefixes.
	for prefix in ("M_D_", "M_", ""):
		if n.startswith(prefix):
			return prefix + n[len(prefix):].split("_", 1)[0]

class Circle:
	def __init__(self, parent, name):
		self.parent, self.name = parent, name
		self.lines = []

	def finalize(self):
		# Just compute our centroid.
		all_indices = set(sum(self.lines, []))
		all_positions = [self.parent.vertices[i] for i in all_indices]
		self.centroid = [sum(x) / float(len(all_positions)) for x in zip(*all_positions)]

class Grid:
	def __init__(self, parent, name):
		self.parent, self.name = parent, name
		self.faces = []

	def finalize(self):
		def estimate_basis(face):
			# Figure out the sizing and spacing of our grid.
			# We do this by finding the projection from any face.
			points = [self.parent.vertices[i] for i in face]
			# Warning: Here I'm going to assume that the points are in CW or CCW order! Unlike in Box,
			# I can't so easily check to verify this assumption, so I'm just going to move forward on faith.
			# TODO: Add an assert here that does the math to check this.
			# First, we subtract corner from the other three.
			a, b, c = [compgeom.sub(p, points[0]) for p in points[1:]]
			# Now, the first corner is our origin, and the remaining three corners have coordinates:
			# \[ a = \hat x          + c_1 \hat z \]
			# \[ b = \hat x + \hat y + c_2 \hat z \]
			# \[ c =          \hat y + c_3 \hat z \]
			# Where $\hat z$ is the out-of-plane vector, and $\hat x$ and$\hat y$ are the in-plane axes of the grid.
			# Therefore, $b - a - c$ corresponds $(c_2 - c_1 - c_3) \hat z$, and gives us a multiple of the out of plane vector.
			out_of_plane = compgeom.sub(b, compgeom.add(a, c))
			# Now, if the four points in this face all have extremely similar out-of-plane heights, then out_of_plane will be almost the zero vector.
			# This is okay -- if this occurs, we simply use $a$ and $c$ as our grid axes.
			# TODO: Revisit what a good crossover point here, and what actually results in stable estimation.
			if compgeom.length(out_of_plane) < 1e-3:
				out_of_plane = compgeom.cross_product(a, b)
			# Normalize the out-of-plane vector, to make the following projection formulae easier.
			out_of_plane = compgeom.to_unit(out_of_plane)
			# Now that we have an out-of-plane vector, we project it out to get our axes.
			project = lambda v: v - compgeom.scale(compgeom.dot_product(out_of_plane, v), out_of_plane)
			axis1, axis2 = compgeom.to_unit(project(a)), compgeom.to_unit(project(c))
			# Now we assert that the two axes have reasonably no dot product.
			assert compgeom.dot_product(axis1, axis2) < 1e-2, "Extracted axes aren't really orthogonal! %r and %r" % (axis1, axis2)
			# Now we canonicalize the basis, so as to allow easy averaging outside this function.
			# Our rule is that 
		# We estimate a basis for each face, and then we average them globally.

# TODO: Possibly migrate some code in Box over here.
def get_square_face_order(coords):
	pass

class Box:
	def __init__(self, parent, name):
		self.parent, self.name = parent, name
		self.faces = []

	def finalize(self):
		assert len(self.faces) in (6, 12), "A box must have either 6 quad faces or 12 triangular faces, not %i faces." % len(self.faces)
		if len(self.faces) == 12:
			print("Rebuilding!")
			# If we have twelve faces then that's because we got triangulated.
			# Undo this here, to rebuild quads.
			assert all(len(face) == 3 for face in self.faces)
			rebuilt_faces = []
			# Here we store a set of the triangles that are already represented in the output by ID.
			# We store by ID in case someone wants to pass in lists (or something else mutable) as triangles.
			already_handled = set()
			for tri in self.faces:
				# Skip triangles we've already found a buddy for.
				if id(tri) in already_handled:
					continue
				# Find the other triangle that shares two verts with this one, and whose third vert is planar with tri.
				for other_tri in self.faces:
					# Again, skip triangles we've already found a buddy for.
					if id(other_tri) in already_handled:
						continue
					# Guarantee that other_tri shares two verts with tri.
					if len(set(other_tri) & set(tri)) != 2:
						continue
					# Guarantee that other_tri and tri are coplanar by checking the absolute value of the inner product of their normals.
					def get_triangle_normal(tri):
						table = self.parent.vertices
						edge1 = compgeom.sub(table[tri[1]], table[tri[0]])
						edge2 = compgeom.sub(table[tri[2]], table[tri[0]])
						return compgeom.to_unit(compgeom.cross_product(edge1, edge2))
					normal1 = get_triangle_normal(tri) # Not hoisted for clarity.
					normal2 = get_triangle_normal(other_tri)
					normal_alignment = abs(compgeom.dot_product(normal1, normal2))
					if normal_alignment < 0.99:
						assert normal_alignment < 0.01, "Bad box with triangles with normals whose inner product is %r. It should always be either 0 or 1." % normal_alignment
						continue
					# Found the buddy triangle!
					# Build a face with the union of the two sets of verts (in the right order!).
					other_tris_extra_vert = set(other_tri) - set(tri)
					assert len(other_tris_extra_vert) == 1, "Buddy triangle should have exactly one new vert! Triangles were: %r and %r." % (tri, other_tri)
					# Pull out the one extra vert that other_tri has that tri doesn't.
					other_tris_extra_vert = list(other_tris_extra_vert)[0]
					# We now rotate tri until tri[0] is the vertex not in other_tri.
					for _ in range(3):
						if tri[0] in other_tri:
							tri = tri[1:] + [tri[0]]
						else:
							break
					else:
						raise Exception("Couldn't rotate %r so its first element wasn't in %r!" % (tri, other_tri))
					# If you think about it for a second, we always want to insert the new vert as the penultimate
					# vert of our new quad in order to preserve the normal, and be a validly ordered quad.
					# Diagram:
					#     tri[0] ----- tri[2]
					#       |            |
					#     tri[1] ----- other_tris_extra_vert
					# This is because other_tri == (tri[1], other_tris_extra_vert, tri[2]) (up to rotation).
					new_face = [tri[0], tri[1], other_tris_extra_vert, tri[2]]
					rebuilt_faces.append(new_face)
					# And finally, mark both of these triangles as already handled so we don't double count anything.
					already_handled.add(id(tri))
					already_handled.add(id(other_tri))
			# Some final sanity checks.
			assert len(rebuilt_faces) == 6, "BUG: Somehow we rebuilt %r faces?" % len(rebuilt_faces)
			self.faces = rebuilt_faces

		# First we find any corner, and its three adjacent corners.
		# These pairs will define the axes of the box.
		key_corner = self.faces[0][0]
		adjacent = set()
		for face in self.faces:
			# If the corner is in the face, find its two adjacent corners.
			if key_corner in face:
				assert len(face) == 4, "Box has face with wrong number of vertices!"
				index = face.index(key_corner)
				# Critical: Here I'm assuming that the corners of the rectangle are listed either in CW or CCW order, but not in a Z-order.
				# To verify this, I check that face[1] - face[0] - face[2] + face[3] == 0
				coords = [self.parent.vertices[i] for i in face]
				error = compgeom.length(compgeom.sub(compgeom.sub(coords[0], coords[1]), compgeom.sub(coords[3], coords[2])))
				assert error < 1e-2, "Ordering on face is bad! %r (error: %r)" % (face, error)
				# Optional: If I start finding I want to import objs that break this convention, uncomment this and comment out the above assert:
				#if error > 1e-2: face[0], face[1] = face[1], face[0]
				for offset in (-1, 1):
					adjacent.add(face[(index + offset) % 4])
		assert len(adjacent) == 3, "A corner on the box has %r adjacent vertices, not three!" % adjacent
		# Now the vectors from the key corner to its three neighbors form the standard basis for the box.
		key_corner_pos = self.parent.vertices[key_corner]
		self.axes = [compgeom.sub(self.parent.vertices[corner], key_corner_pos) for corner in adjacent]
		# If the determinant is negative then we need to swap to axes to produce a valid rotation matrix.
		if compgeom.determinant(self.axes) < 0.0:
			self.axes[0], self.axes[1] = self.axes[1], self.axes[0]
		# Compute the actual dimensions along the axes.
		self.dimensions = list(map(compgeom.length, self.axes))
		# Compute the centroid of the whole box by averaging all eight corners.
		all_corners_indices = set(sum(self.faces, []))
		assert len(all_corners_indices) == 8, "Box doesn't have eight corners: %r" % all_corners_indices
		all_corners = [self.parent.vertices[i] for i in all_corners_indices]
		self.centroid = [sum(x) / 8.0 for x in zip(*all_corners)]
		#print self.centroid
		# XXX: TODO: I changed this convention back! Fix up the comments to reflect this...
		# Compute the transpose of the rotation matrix that brings world coordinates to our coordiantes.
		# That is, the action of left-multiplying by this matrix (assuming that self.rotation_matrix[i][j] is the $A_{ij}$ entry)
		# has the effect of axis aligning this box to the global axes. Whew, what a mouthful to define.
		self.rotation_matrix = list(zip(*compgeom.normalize_rows(self.axes)))
		self.rotation_axis, self.rotation_angle = compgeom.so3_to_axis_angle(self.rotation_matrix)
		#print self.rotation_axis, self.rotation_angle / math.pi

class GenericMesh:
	def __init__(self, parent, name):
		self.parent, self.name = parent, name
		self.faces = []
		self.faces_texture_indices = []
		self.faces_normal_indices = []
		self.edges = []
		self.triangles = []
		self.triangles_texture_coords = []

	def to_box(self):
		box = Box(self.parent, self.name)
		box.faces = self.faces[:]
		box.finalize()
		return box

	def finalize(self):
		# Compute a centroid by averaging all referenced coordinates.
		# This is not a true COM or other form of centroid, just a cheap hack for getting an approximate center of volumes!
		all_coords = sum([[self.parent.vertices[i] for i in face] for face in self.faces], [])
		self.centroid = [sum(x) / float(len(all_coords)) for x in zip(*all_coords)]

class Door:
	def __init__(self, name, coords):
		self.name, self.coords = name, coords
		assert len(coords) == 4
		self.centroid = [sum(x) / 4.0 for x in zip(*self.coords)]
		# We now compute a "local axes matrix" for the door.
		# This matrix's three columns are the door's normal, "up vector", and the cross product of the normal and "up vector" respectively.
		# It is guaranteed to be an orthonormal matrix with determinant +1.

	def finalize(self):
		# Here I assume a counter-clockwise ordering on the vertices on the face.
		# TODO: Examine the normals on the vertices, and use them to verify this assumption.
		vec1 = compgeom.sub(self.coords[1], self.coords[0])
		vec2 = compgeom.sub(self.coords[3], self.coords[0])
		normal = compgeom.to_unit(compgeom.cross_product(vec1, vec2))
		# Now, to find up, we find which of vec1 and vec2 has the highest magnitude dot product with (0, 0, 1).
		# This is the most "uppity" (or highest upness) vector.
		most_uppity = max((vec1, vec2), key=lambda v: abs(v[2]))
		# Now flip the vector around if it's the wrong way around.
		if most_uppity[2] < 0:
			most_uppity = compgeom.scale(-1, most_uppity)
		# TODO: If I want to handle doors that aren't vertically aligned then I'm going to need to be a bit more clever about finding this vector.
#		up = [0, 0, 1]
		up = most_uppity
		print("Up vector:", up)
		# The normal vector and up should have almost no overlap.
		assert -0.01 < compgeom.dot_product(normal, up) < 0.01, "A door's implicit normal has too large of a dot product with up. Do you have a non-vertical door?"

		columns = [normal, up, compgeom.cross_product(normal, up)]
		columns = compgeom.normalize_rows(columns)
		self.local_axes_matrix = list(zip(*columns))
		det = compgeom.determinant(self.local_axes_matrix)
		assert 0.99 < det < 1.01, "BUGBUGBUG: A Door's local axes matrix ended up with a bad determinant of %r" % det

class Wavefront:
	DOOR_SIZE = 2.0

	def face_qualifies_as_door_shaped(self, face):
		# Do a series of tests to check if this face is a door.
		# 1) It must have four faces. No triangulated doors for now! Delete that diagonal edge!
		if len(face) != 4:
			return False
		# 2) It must be approximately a parallelogram.
		coords = [self.vertices[i] for i in face]
		error = compgeom.length(compgeom.sub(compgeom.sub(coords[0], coords[1]), compgeom.sub(coords[3], coords[2])))
		if error > 1e-2:
			return False
		# 3) The two spanning vectors must be basically orthogonal.
		span1, span2 = compgeom.sub(coords[1], coords[0]), compgeom.sub(coords[3], coords[0])
		if compgeom.dot_product(span1, span2) > 1e-2:
			return False
		# 4) Each spanning vector must have length basically equal to one.
		if any(abs(self.DOOR_SIZE - compgeom.length(span)) > 1e-2 for span in (span1, span2)):
			return False
		# If all tests pass, then it is door shaped.
		return True

	def load(self, path):
		self.metadata = {"doors": {}}
		# First we check to see if a metadata JSON file can be found.
		if path.endswith(".obj"):
			json_path = path[:-4] + ".json"
			if os.path.exists(json_path):
				# Let the exception be raised if this fails.
				# If the .json file exists, but we can't load it for some reason this deserves a crash.
				with open(json_path) as json_f:
					self.metadata = json.load(json_f)
		f = open(path)

		# These None placeholders are to make the lists effectively one-indexed.
		# It also functions nicely to make stuff blow up if an index of zero ever occurs.
		# Otherwise, if I simply subtracted one then a zero index would "work" silently.
		self.vertices = [None]
		self.texture_coords = [None]
		self.normals = [None]
		self.objects = []
		self.object_map = {}
		for line in f:
			line = line.strip()
			# Simple heuristic for comments. I'm unsure if the comment character can be later in.
			if line.startswith("#"):
				continue
			args = line.split(" ")
			if args[0] == "v":
				assert len(args) == 4
				self.vertices.append(AXIS_SWAP(list(map(float, args[1:]))))
			elif args[0] == "vt":
				assert len(args) == 3
				values = list(map(float, args[1:]))
				# XXX: Why is it necessary to invert the v coordinate?
				# Double check all the conventions...
				values = values[0], 1-values[1]
				self.texture_coords.append(values)
			elif args[0] == "vn":
				assert len(args) == 4
				self.normals.append(AXIS_SWAP(list(map(float, args[1:]))))
			elif args[0] == "o":
				assert len(args) == 2
				# Check to see if we're creating a circle.
				self.objects.append(GenericMesh(self, args[1]))
				# Save this object by name as well.
				self.object_map[args[1]] = self.objects[-1]
			elif args[0] in ("f", "l"):
				obj = self.objects[-1]
				vertex_indices = []
				texture_indices = []
				normal_indices = []
				for arg in args[1:]:
					for i, field in enumerate(arg.split("/")):
						# We need this guard statement to support missing texture indices in the syntax 1//2
						if not field:
							continue
						[vertex_indices, texture_indices, normal_indices][i].append(int(field))
#				print "Face:", vertex_indices
				if args[0] == "f":
					obj.faces.append(vertex_indices)
					obj.faces_texture_indices.append(texture_indices or None)
					obj.faces_normal_indices.append(normal_indices or None)
				else:
					# Lines don't get texture or normal information saved for now.
					# Perhaps later I should change this.
					#obj.lines.append(vertex_indices)
					print("Edge in imported file...")
			else:
				print("Unknown:", args)
				pass
		# Now we do all the extra computations.
		for obj in self.objects:
			obj.finalize()

		# Now we unpack all the various objects into triangle sets.
		# At the same time we filter out "banned triangles" that are part of a door.
		# Each door is a square quad, and we don't know which of the two possible ways it will be triangulated,
		# so for efficiency we precompute every possible banned triangle in every possible vertex order.
		banned_triangles = set()
		vertex_tuple_to_index = {tuple(vertex): i for i, vertex in enumerate(self.vertices) if vertex != None}
		for door_name, door_points in self.metadata["doors"].items():
#!!			# XXX: This is where inexactness of floats will be turbo obnoxious... :(
			# Find the indices of the door's four corners.
			# There are some difficulties with doing this directly, so first I refer the reader to
			#   ./release/scripts/addons/io_scene_obj/export_obj.py
			# from the Blender source, as of the commit e074554f8ffe16e5693c7f5ab33898f902166fad.
			# This addon handles exporting OBJ files. Now, inconveniently, on line 498 of said
			# file the vertices are written out using the format string 'v %.6f %.6f %.6f\n'.
			# This seriously truncates the precision of the OBJ file's coordinates.
			#
			# By looking up in vertex_tuple_to_index we are checking for an exact bit match of floats.
			# Therefore we have to apply the same mapping as the OBJ exporter applies to coordinates
			# to get an exact bit match. Thus, the following strange code.
			precision_truncated_points = [tuple(float("%.6f" % x) for x in point) for point in door_points]
			door_indices = [vertex_tuple_to_index[tup] for tup in precision_truncated_points]
			# Ban every permutation of the four possible triangles formed by omitting any corner of the quad.
			for subset in itertools.combinations(door_indices, 3):
				for permutation in itertools.permutations(subset):
					banned_triangles.add(permutation)

		print("Got %i banned triangles." % len(banned_triangles))

		# Make a single list with all the geometry triangles in it.
		self.triangles = []
		self.triangles_texture_coords = []
		# All marker locations.
#		self.marker_locations = {}
#!!		# Simultaneously we're going to find all unit square faces that might be doors.
#!!		door_candidates = []
		for obj in self.objects:
			# XXX: I got rid of the idea that objects that begin with M_ aren't included in the geometry.
			# XXX: This is now handled via the metadata file. Thus, the following comment is commented out.
#			# Objects whose name begins with M_ are markers and aren't included in the geometry.
#			if obj.name.startswith("M_"):
#				self.marker_locations[extract_name(obj.name)] = obj.centroid
#				continue
			for face, texture_indices in zip(obj.faces, obj.faces_texture_indices):
#				if self.face_qualifies_as_door_shaped(face):
#					# We've detected a possible door!
#					door_candidates.append(Door(self, face))
#				else:
				if tuple(face) in banned_triangles:
					# Don't triangulate faces that are also part of doors!
					print("Skipping adding this face to the object's geometry, because it's part of a door:", face)
					continue
				# Add this face's triangles into the level geometry.
				# First we have to check if the face has corresponding
				face_as_triangles, face_texture_coords = self.triangulate(face, texture_indices)
				obj.triangles.extend(face_as_triangles)
				obj.triangles_texture_coords.extend(face_texture_coords)
			self.triangles.extend(obj.triangles)
			self.triangles_texture_coords.extend(obj.triangles_texture_coords)
		assert len(self.triangles) == len(self.triangles_texture_coords), "BUGBUGBUG. Texture coords and regular coords lists got out of sync."

		# Find all door markers, and check for doors that fit.	
		self.doors_by_name = {door_name: Door(door_name, face) for door_name, face in self.metadata["doors"].items()}
		self.doors = list(self.doors_by_name.values())
		for door in self.doors:
			door.finalize()

		"""
		self.doors = set()
		self.doors_by_name = {}
#		for obj in self.objects:
		for marker_name, marker_location in self.metadata["marker_locations"].iteritems():
			if not marker_name.startswith("M_D_"):
				continue
			print "Testing for:", marker_name
			# Find if a door candidate matches this location closely.
			already_have_one = False
			for door in door_candidates:
				if compgeom.l2(door.centroid, marker_location) < 1e-2:
					if already_have_one:
						raise Exception("Bad map file! Door marker %r matches multiple faces. Do you have doubled up geometry?" % marker_name)
					print "Validated door:", marker_name, door
					# Link up the door's name -- we've identified this door.
					door.name = marker_name
					self.doors.add(door)
					self.doors_by_name[door.name] = door
					already_have_one = True
		self.doors = list(self.doors)
		# Do the normal and axes computations after we have names, because
		# we need the names to correctly index into the manual tweaks.
		for door in self.doors:
			door.finalize()
"""

		# Compute our AABB.
		self.aabb = compgeom.compute_aabb_of_triangles(self.triangles)
		print("Found a total of", len(self.doors), "doors, and", len(self.triangles), "triangles.")

	def triangulate(self, vertex_indices, texture_indices=None):
		triangles = []
		triangles_texture_coords = []
		for i in range(1, len(vertex_indices)-1):
			indices = [vertex_indices[0], vertex_indices[i], vertex_indices[i+1]]
			triangles.append(list(map(self.vertices.__getitem__, indices)))
			if texture_indices != None:
				indices = [texture_indices[0], texture_indices[i], texture_indices[i+1]]
				triangles_texture_coords.append(list(map(self.texture_coords.__getitem__, indices)))
		if texture_indices == None:
			return triangles, [((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))] * len(triangles)
		else:
			return triangles, triangles_texture_coords

if __name__ == "__main__":
	w = Wavefront()
	w.load("maps/untitled.obj")

