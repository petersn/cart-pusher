#!/usr/bin/python

import os
import link
import wavefront_importer

# Scan the models directory to find potential model/texture pairs to load
potential_models = {}
for dirpath, dirnames, filenames in os.walk("models"):
	for f in filenames:
		if f.endswith(".obj"):
			print "Found:", os.path.join(dirpath, f)
			texture_path = None
			potential_texture_path = os.path.join(dirpath, f[:-4] + ".png")
			if os.path.isfile(potential_texture_path):
				texture_path = potential_texture_path
				print "Texture:", texture_path
			potential_models[f[:-4]] = (os.path.join(dirpath, f), texture_path)

model_cache = {}

class ModelSubObject:
	def __init__(self, obj, metadata):
		self.obj, self.metadata = obj, metadata
		# TODO: When I work on distributing this, make sure the paths work out appropriately here.
		self.name = self.obj.name
		self.triangles = self.obj.triangles
		self.triangles_texture_coords = self.obj.triangles_texture_coords
		# XXX: TODO: FIXME: This is super duper hacky, but for just now...
		# Rather than fix my addon to properly set relative paths to textures, simply fix up the paths right here.
		# (Timestamp, so I can be shamed for how long I leave this hack: Mon Jan 23 06:58:15 EST 2017)
		texture_path = self.metadata.get("textures", {}).get(self.name, "data/stone.png")
		texture_path = texture_path.replace("/home/snp/proj/cart-pusher/", "./")
		self.t = link.Texture(texture_path)
		self.arrays = link.pack_triangles(self.obj.triangles, self.obj.triangles_texture_coords)

	def render(self):
		link.draw_triangles(self.arrays, self.t.index)

class Model:
	def __init__(self, name, obj_path, texture_path=None):
		self.name, self.obj_path, self.texture_path = name, obj_path, texture_path
		self.w = wavefront_importer.Wavefront()
		self.w.load(self.obj_path)
#		self.t = link.get_texture(os.path.join("data", "stone.png"))
#		if texture_path != None:
#			self.t = link.Texture(self.texture_path)
#			print "Model", name, "using texture:", self.t.index
#		else:
#			print "Model", name, "using default texture:", self.t.index
#		# Pack triangles into arrays for fast rendering.
#		self.obj = self.w.objects[0]
#		self.arrays = link.pack_triangles(self.obj.triangles, self.obj.triangles_texture_coords)
		self.sub_objects = [ModelSubObject(obj, self.w.metadata) for obj in self.w.objects]
		# XXX: Obnoxious legacy. Refactor this.
		self.obj = self.sub_objects[0]

	def render(self):
		for obj in self.sub_objects:
			obj.render()
		#link.draw_triangles(self.arrays, self.t.index)

def get_model(model_name):
	if model_name not in model_cache:
		model = Model(model_name, *potential_models[model_name])
		model_cache[model_name] = model
	return model_cache[model_name]

