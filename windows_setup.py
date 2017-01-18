# py2exe script for Windows builds

import os
from distutils.core import setup
import py2exe

global_exclude = [
	"generate.py",
	"whitrabt.ttf",
]

def nab(directory):
	return [(root, [os.path.join(root, f) for f in files if f not in global_exclude]) for root, dirs, files in os.walk(directory)]

setup(
	console=["game.py"],
	options={
		"py2exe": {
#			"bundle_files": 1,
#			"compressed": True,
			"excludes": ["_ssl", "doctest", "pdb", "unittest", "difflib", "inspect", "pygame", "pygame.*"],
		},
	},
	data_files=[
		(".", [
			"windows_build/dlls/zlib1.dll",
			"windows_build/dlls/SDL.dll",
			"windows_build/dlls/libpng12-0.dll",
			"windows_build/dlls/opengl32.dll",
			"windows_build/dlls/glu32.dll",
			"libeg.dll"
		]),
	] + nab("data") + nab("bitfont") + nab("maps"),
)

