#!/usr/bin/python
"""
compgeom - some linear algebra and other computational geometry

Note: None of the routines in this are at all performant, and should not be used in performance-critical regions.
"""

import math, random
#import numpy

identity3x3 = [[+(i == j) for i in range(3)] for j in range(3)]

# :(
EPSILON = 1e-15

def sub(a, b):
	return a[0] - b[0], a[1] - b[1], a[2] - b[2]

def add(a, b):
	return a[0] + b[0], a[1] + b[1], a[2] + b[2]

def scale(c, a):
	return c * a[0], c * a[1], c * a[2]

def length(v):
	return (v[0]**2 + v[1]**2 + v[2]**2)**0.5

def to_unit(v):
	return scale(1.0/length(v), v)

def matrix_product(A, B):
	# Transpose B, to get column vectors we can readily take inner products with.
	B = list(zip(*B))
	return [[sum(i*j for i, j in zip(v1, v2)) for v1 in B] for v2 in A]

def matrix_vector_product(A, x):
	return [sum(i*j for i, j in zip(row, x)) for row in A]

def quaternion_product(q1, q2):
	"""quaternion_product(q1, q2) -> q1 * q2
	Uses the convention that a quaternion is a tuple of components, (1, i, j, k) respectively.
	"""
	a1, b1, c1, d1 = q1
	a2, b2, c2, d2 = q2
	return (
		a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2,
		a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
		a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2,
		a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
	)

def quaternion_norm(q):
	return sum(i**2 for i in q)**0.5

def axis_angle_to_quaternion(axis_angle):
	axis, angle = axis_angle
	axis = to_unit(axis)
	cos, sin = math.cos(angle / 2.0), math.sin(angle / 2.0)
	return cos, sin * axis[0], sin * axis[1], sin * axis[2]

def quaternion_to_axis_angle(quaternion):
	axis = quaternion[1:]
	axis_length = length(axis)
	angle = 2 * math.atan2(axis_length, quaternion[0])
	if axis_length < EPSILON:
		return (1, 0, 0), 0
	return to_unit(axis), angle

def compose_axis_angle_rotations(aa1, aa2):
	"""compose_axis_angle_rotations(aa1, aa2) -> aa3
	Gives an axis and angle aa3 that corresponds to rotation by aa1 followed by aa2.
	All each "aa" must be of the form ((axis[0], axis[1], axis[2]), angle).
	"""
	q1, q2 = map(axis_angle_to_quaternion, [aa1, aa2])
#	print "Norms:", map(quaternion_norm, (q1, q1))
	# Note that q3 = q2 q1, so that q1 happens first.
	q3 = quaternion_product(q2, q1)
#	print "Final norm:", quaternion_norm(q3)
	return quaternion_to_axis_angle(q3)

def apply_axis_angle_rotation_to_vector(axis_angle, vector):
	# Uses Rodrigues' formula.
	axis, angle = axis_angle
	axis = to_unit(axis)
	sin, cos = math.sin(angle), math.cos(angle)
	output = scale(cos, vector)
	output = add(output, scale(sin, cross_product(axis, vector)))
	output = add(output, scale((1.0 - cos) * dot_product(axis, vector), axis))
	return output

def transpose(m):
	return list(zip(*m))

def determinant(mat):
	(a, b, c), (d, e, f), (g, h, i) = mat
	return a*e*i +b*f*g + c*d*h - c*e*g - b*d*i - a*f*h

def dot_product(a, b):
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def cross_product(a, b):
	a1, a2, a3 = a
	b1, b2, b3 = b
	return (a2 * b3 - a3 * b2), (a3 * b1 - a1 * b3), (a1 * b2 - a2 * b1)

def normalize_vector(vec):
	inv_norm = sum(i**2 for i in vec)**-0.5
	return [i * inv_norm for i in vec]

def normalize_rows(mat):
	return [normalize_vector(vec) for vec in mat]

def l2(a, b):
	return length(sub(a, b))

def lerp(coef, a, b):
	return add(scale(1.0 - coef, a), scale(coef, b))

def trace(m):
	return sum(row[i] for i, row in enumerate(m))

def so3_to_quaternion(m):
#	m = numpy.array(m)
#	non_orthogonality = numpy.linalg.norm((m.dot(m.transpose()) - numpy.eye(3)).reshape((9,)))
#	assert non_orthogonality < 1e-3, "Fed in a non SO3 matrix to so3_to_quaternion()."
	print("Input so3:", m)
	for case_number, transform in enumerate([(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]):
#		if numpy.trace(numpy.diag(transform) * m) < 0:
#			continue
		if sum(value * m[i][i] for i, value in enumerate(transform)) < 0:
			continue
#		m = m.dot(numpy.diag(transform))
		m = [[x * y for x, y in zip(row, transform)] for row in m]
		quaternion_real = (1 + trace(m))**0.5 / 2
		scale = 1.0 / (4 * quaternion_real)
		result = [
			quaternion_real,
			(m[2][1] - m[1][2]) * scale,
			(m[0][2] - m[2][0]) * scale,
			(m[1][0] - m[0][1]) * scale,
		]
		fixup = [0, 0, 0, 0]
		fixup[case_number] = 1
		result = quaternion_product(result, fixup)
		return result
	raise ValueError("What happened?")

def so3_to_axis_angle(m):
	return quaternion_to_axis_angle(so3_to_quaternion(m))

def random_so3r():
	"""random_so3r() -> Returns a random element of $SO_3(\mathbb{R})$.

	Not uniform with respect to the usual measure! Mostly for debugging.
	"""
	v1, v2 = normalize_rows([[random.uniform(-1, 1) for i in range(3)] for j in range(2)])
	v3 = cross_product(v1, v2)
	v2 = cross_product(v3, v1)
	so3r = [v1, v2, v3]
	# We now have three orthogonal vectors.
#	so3r = [[random.uniform(-1, 1) for i in range(3)] for j in range(3)]
#	so3r = [[so3r[i][j] + so3r[j][i] for i in range(3)] for j in range(3)]
	so3r = normalize_rows(so3r)
	if determinant(so3r) < 0:
#		print("Bad determinant")
		so3r = [[so3r[i][j] * (-1 if i == 0 else 1) for i in range(3)] for j in range(3)]
#	print matrix_product(so3r, transpose(so3r))
	return so3r

def compute_aabb(points):
	coords = list(zip(*points))
	minima, maxima = [map(f, coords) for f in (min, max)]
	return (minima, maxima)

def compute_aabb_of_triangles(triangles):
	points = sum(triangles, [])
	return compute_aabb(points)

def point_in_aabb(point, aabb):
	return all(aabb[0][i] <= point[i] <= aabb[1][i] for i in range(3))

