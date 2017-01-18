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

class Model:
	def __init__(self, name, obj_path, texture_path=None):
		self.name, self.obj_path, self.texture_path = name, obj_path, texture_path
		self.w = wavefront_importer.Wavefront()
		self.w.load(self.obj_path)
		self.t = link.get_texture(os.path.join("data", "stone.png"))
		if texture_path != None:
			self.t = link.Texture(self.texture_path)
			print "Model", name, "using texture:", self.t.index
		else:
			print "Model", name, "using default texture:", self.t.index
		# Pack triangles into arrays for fast rendering.
		self.obj = self.w.objects[0]
		self.arrays = link.pack_triangles(self.obj.triangles, self.obj.triangles_texture_coords)

	def render(self):
		link.draw_triangles(self.arrays, self.t.index)

def get_model(model_name):
	if model_name not in model_cache:
		model = Model(model_name, *potential_models[model_name])
		model_cache[model_name] = model
	return model_cache[model_name]

