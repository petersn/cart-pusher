#!/usr/bin/python
# cart-pusher client

import link
import server
import compgeom
import simulation
import os_interface
from config import *

import math, time, random, socket, struct, thread, json, Queue
import numpy as np

class NetworkManager:
	def __init__(self, host, port):
		self.host, self.port = host, port
		self.network_event_queue = Queue.Queue()
		self.query_response_queue = Queue.Queue()

	def poll_thread(self):
		while True:
			# Get a type and length.
			message_type = self.sock.recv(1)

			# Message type: Query response.
			if message_type == "r":
				s = server.get_string(self.f)
				print repr(s)
				payload = json.loads(s)
				self.query_response_queue.put(payload)
			# Message type: Event.
			elif message_type == "e":
				payload = server.get_string(self.f)
				self.network_event_queue.put(payload)
			# Message type: State.
			elif message_type == "s":
				payload = server.get_string(self.f)
				payload = json.loads(STATE_DECOMPRESS(payload))
				self.network_event_queue.put(payload)
			else:
				raise ValueError("Network message type %r not valid! Framing error?" % message_type)

	def query(self, data):
		self.f.write("q")
		server.send_string(self.f, json.dumps(data))
		return self.query_response_queue.get()

	def player_input(self, data):
		self.f.write("i")
		server.send_string(self.f, json.dumps(data))

	def connect(self):
		# Try to actually connect to our remote host.
		self.sock = socket.create_connection((self.host, self.port))
		self.f = self.sock.makefile()

		# Launch our asynchronous polling thread that listens for messages.
		thread.start_new_thread(self.poll_thread, ())

		# Get an initial ping message just to confirm everything is working.
		reply = self.query({"type": "ping"})
		assert reply == "pong"

def shutdown():
	link.graphics_close()
	# Send a disconnect message to the server.
	network_manager.sock.sendall("x")
	exit()

consonants = [""] + list("qwrtpsdfghjklzxcvbnm")
vowels = "aoeui"
name_syllables = [c1 + v + c2 for c1 in consonants for v in vowels for c2 in consonants]

def random_name():
	return "".join(random.sample(name_syllables, 2))

def main(game_options):
	global network_manager
	# Connect to the game server.
	network_manager = NetworkManager(game_options.host, game_options.port)
	network_manager.connect()

	player_name = game_options.name
	# Generate a random player name if the user didn't set one.
	if player_name == None:
		player_name = random_name()

	my_player_ent_id = network_manager.query({
		"type": "introduce_player",
		"name": player_name,
	})

	print "Our player's ent_id:", my_player_ent_id

	link.graphics_init(game_options.fullscreen)

	keys_held = set()
	last_sim_physics_update = time.time()
	smoothed_fps = 0.0

	sim = simulation.GameSimulation(is_server_side=False)
#	sim.add_entity(simulation.Cart, [(0, 3, 0)])
	player = None
	smoothed_xyz = np.array([0, 0, 0])

	camera_orientation = [0.0, 0.0]
	def adjust_camera(horizontal, vertical):
		camera_orientation[0] += horizontal
		camera_orientation[1] += vertical
		camera_orientation[1] = min(math.pi/2, max(-math.pi/2, camera_orientation[1]))

	while True:
		# Pull our player object out, if it's been synced yet.
		if my_player_ent_id in sim.entities:
			player = sim.entities[my_player_ent_id]
			player.is_actively_controlled_player = True
		else:
			player = None

		commands_this_frame = set()
		def issue_command(command):
			if player:
				player.try_to_command(command)
				commands_this_frame.add(command)

		# Process the SDL event queue.
		while link.get_event():
			if link.last_event.type == link.E_QUIT or (link.last_event.type == link.E_KEY_DOWN and link.last_event.key == 27):
				shutdown()
			elif link.last_event.type == link.E_KEY_DOWN:
				keys_held.add(link.last_event.key)
				if link.last_event.key == link.K_SPACE:
					issue_command("jump")
			elif link.last_event.type == link.E_KEY_UP:
				if link.last_event.key in keys_held:
					keys_held.remove(link.last_event.key)
			elif link.last_event.type == link.E_MOUSE_DOWN:
				keys_held.add(("mouse", link.last_event.button))
			elif link.last_event.type == link.E_MOUSE_UP:
				evt = ("mouse", link.last_event.button)
				if evt in keys_held:
					keys_held.remove(evt)

		# Get the mouse position, and use it to adjust the camera angles.
		x_offset = link.mouse_x.value - link.screen_width.value/2
		y_offset = link.mouse_y.value - link.screen_height.value/2
		if game_options.fullscreen or link.K_L_SHIFT in keys_held or link.K_R_SHIFT in keys_held:
#			if player:
				adjust_camera(x_offset * CAMERA_FACING_RATE, y_offset * CAMERA_TILT_RATE)
		if game_options.fullscreen:
			link.warp_mouse(link.screen_width.value/2, link.screen_height.value/2)
		if player:
			player.facing, player.tilt = camera_orientation
			xyz = player.get_xyz()
			alpha = CAMERA_SMOOTHING_FACTOR
			smoothed_xyz = (1 - alpha) * smoothed_xyz + alpha * xyz
			link.camera_x.value = smoothed_xyz[0]
			link.camera_y.value = smoothed_xyz[1]
			link.camera_z.value = smoothed_xyz[2] + player.eye_height
		link.camera_tilt.value = math.degrees(camera_orientation[1])
		link.camera_facing.value = math.degrees(camera_orientation[0])

		move_vector = [0.0, 0.0]
		if ord("a") in keys_held:
			move_vector[1] -= 1
		if ord("d") in keys_held or ord("e") in keys_held:
			move_vector[1] += 1
		if ord("w") in keys_held or ord(",") in keys_held:
			move_vector[0] += 1
		if ord("s") in keys_held or ord("o") in keys_held:
			move_vector[0] -= 1
		if player and USE_MOVE_PREDICTION:
			player.input_motion_vector = move_vector

		if ("mouse", 1) in keys_held:
			issue_command("m1")

		# Send an update to the server.
		if player:
			player_input = {
				"facing": player.facing,
				"tilt": player.tilt,
#				"motion_vector": player.input_motion_vector,
				"motion_vector": move_vector,
				"commands": list(commands_this_frame),
			}
			network_manager.player_input(player_input)

		# Get events from our queue.
		try:
			while True:
				event = network_manager.network_event_queue.get_nowait()
				# Sorry, to the person who will be bothered by this dispatch.
				# I realize that the JSON could encode a string...
				if isinstance(event, str):
					sim.apply_serialized_patches(event)
				else:
					sim.apply_serialized_state(event)
		except Queue.Empty:
			pass

		# Update the simulation.
#		if player:
#			player.apply_forces()
		now = time.time()
		elapsed = now - last_sim_physics_update
		sim.step(min(0.1, elapsed))
		last_sim_physics_update = now

		alpha = 0.05
		smoothed_fps = (1.0/max(0.001, elapsed)) * alpha + smoothed_fps * (1 - alpha)

		# Begin drawing the frame.
		link.begin_frame()
		sim.draw()
		link.begin_overlay()

		status_string = ""
		cart = sim.get_ent(lambda x: isinstance(x, simulation.Cart))
		if cart:
			status_string += "Cart state: %s\n" % cart.push_state

		if player:
			status_string += "HP: %3i/100\n" % player.hp
			status_string += "Ammo: %2s/%s" % (player.ammo, player.max_ammo)
#			status_string += " [%2i]" % player.ammo_claim_patches
			if player.reload_cooldown > 0.0:
				status_string += " [%-20s]\n" % ("*" * int(20.0 * player.reload_cooldown / player.reload_interval),)
			else:
				status_string += "\n"
		else:
			respawn_handle = sim.get_ent(lambda x: isinstance(x, simulation.RespawnHandle) and x.player_ent_id == my_player_ent_id)
			if respawn_handle:
				link.draw_centered_text("===========\n| D E A D |\n===========\n" + " Time: %.1f" % max(0.0, respawn_handle.respawn_cooldown))

		link.set_color(0, 0, 0, 1)
#		status_string = "[%5.2f fps %.2f MiB %.1f%% CPU]\n%s" % (smoothed_fps, os_interface.memory_usage()/(2**20.0), 100 * os_interface.cpu_usage(), status_string)
		link.draw_bordered_text((10, 10), status_string)
		link.end_frame()

if __name__ == "__main__":
	import argparse
	parser = argparse.ArgumentParser(prog="client.py", description="cart-pusher")
	parser.add_argument("--name", default=None, help="Player name to use in game.")
	parser.add_argument("--port", type=int, default=58328, help="Port to connect on.")
	parser.add_argument("--host", default="localhost", help="Remote host of server.")
	parser.add_argument("--no-fullscreen", dest="fullscreen", action="store_const", const=False, default=True, help="Don't render fullscreen.")
	args = parser.parse_args()
	print "=== cart-pusher client ==="
	main(args)

