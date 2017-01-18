#!/usr/bin/python

import random, math, time, numpy

def to_unit(v):
	return v / numpy.linalg.norm(v)

class LevelGenerator:
	cone_const = 5.0
	height_change_probability = 0.2

	def __init__(self, seed, segment_count=40):
		self.random = random.Random(seed)
		self.points = [numpy.array([0.0, 0.0])]
		self.direction = 0.0
		self.direction_accum = 0.0
		self.widths = [6.0]
		self.heights = [0.0]
		self.total_length = 0.0

		for segment_index in xrange(segment_count):
			do_slope = segment_index > 3 and self.random.random() < self.height_change_probability
			self.direction_accum *= 0.8
			self.direction_accum += self.random.normalvariate(0, 0.4)
			self.direction_accum += -self.direction * 0.3
			self.direction += self.direction_accum
			self.direction = max(-self.cone_const * math.pi/2, min(self.cone_const * math.pi/2, self.direction))
			length = self.random.uniform(4, 12)
			if do_slope:
				length *= 2
			self.total_length += length

			xy = numpy.array([math.cos(self.direction) * length, math.sin(self.direction) * length])
			self.points.append(xy + self.points[-1])

			# Manage width changes.
			new_width = self.widths[-1] + self.random.normalvariate(0, 4)
			new_width = max(8, min(16, new_width))
			self.widths.append(new_width)

			# Manage height changes.
			height_delta = 0
			if do_slope:
				height_delta = 0.2 * self.random.choice((-1, 1)) * length

			self.heights.append(self.heights[-1] + height_delta)

		# Renormalize so that self.points[1] is at the origin, for convenience.
		value = self.points[1]
		for i in xrange(len(self.points)):
			self.points[i] -= value

	def get_xyz_theta_on_path(self, distance):
		i = 1
		while i < len(self.points) - 1:
			curr, succ = self.points[i], self.points[i+1]
			length = numpy.linalg.norm(succ - curr)
			theta = math.atan2(succ[1] - curr[1], succ[0] - curr[0])
			if length > distance:
				alpha = distance / length
				xy = (1.0 - alpha) * curr + alpha * succ
				h = (1.0 - alpha) * self.heights[i] + alpha * self.heights[i+1]
				return numpy.array([xy[0], xy[1], h]), theta
			distance -= length
			i += 1
		return numpy.array([self.points[-1][0], self.points[-1][1], self.heights[-1]]), theta

	def generate_triangles(self):
		self.edges = []
		for i in xrange(1, len(self.points)-1):
			pred = self.points[i-1]
			curr = self.points[i]
			succ = self.points[i+1]

			direction = to_unit(to_unit(succ - curr) + to_unit(curr - pred))
			width_direction = numpy.array([direction[1], -direction[0]])
			self.edges.append([curr + width_direction * self.widths[i] * v for v in (-0.5, 0.5)])

		self.triangles = []
		self.triangles_texture_coords = []

		for i in xrange(len(self.edges)-1):
			(left, right), (left_next, right_next) = self.edges[i], self.edges[i+1]
			h, h_next = self.heights[i+1], self.heights[i+2]
			f = lambda point, height: (point[0], point[1], height)
			def make_quad(a, b, c, d, cd_height, flip_normal=False):
				# Make a quad out of:
				# d -- c
				# |    |
				# a -- b
				# Here `a` and `b` have a height of zero, and `c` and `d` have a height of `cd_height`.
				self.triangles.append([f(a, h), f(c, h_next + cd_height), f(d, h + cd_height)])
				self.triangles.append([f(a, h), f(b, h_next), f(c, h_next + cd_height)])
				self.triangles_texture_coords.append([(0,0), (1, 1), (0, 1)])
				self.triangles_texture_coords.append([(0,0), (1, 0), (1, 1)])
				if flip_normal:
					for l in (self.triangles, self.triangles_texture_coords):
						for i in (-1, -2):
							l[i] = l[i][::-1]
			# Make a floor.
			make_quad(right, right_next, left_next, left, 0)
			# Make a left wall.
			make_quad(left, left_next, left_next, left, 10)
			# Make a right wall.
			make_quad(right, right_next, right_next, right, 10, flip_normal=True)

		return self.triangles

	def show_level(self):
		import pygame
		pygame.init()
		width, height = 1280, 720
		screen = pygame.display.set_mode((width, height))

		screen.fill((255, 255, 255))
		for i, j, width in zip(self.points, self.points[1:], self.widths):
			SCALE = lambda (x, y): (x*1 + 100, y*1 + height/2)
			width_dir = numpy.array([j[1] - i[1], i[0] - j[0]])
			width_dir /= numpy.linalg.norm(width_dir)
			pygame.draw.line(screen, (0, 0, 0), SCALE(i), SCALE(j))
			dumps = [i + width_dir * width * v for v in (-0.5, 0.5)]
			pygame.draw.line(screen, (0, 0, 0), SCALE(dumps[0]), SCALE(dumps[1]))

		while True:
			for event in pygame.event.get():
				if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == 27):
					pygame.quit()
					return
			pygame.display.update()
			time.sleep(0.1)

if __name__ == "__main__":
	lg = LevelGenerator(10)
	lg.show_level()

