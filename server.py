#!/usr/bin/python
# cart-pusher game server

import sys, socket, SocketServer, struct, thread, threading, time, json, Queue, fractions
import link
import simulation
from config import *

PORT = 58328

class Server(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
	allow_reuse_address = True

global_player_handlers = []
global_lock = threading.Lock()

global_data_stats_lock = threading.Lock()
global_stats = {"data_sent": 0}

def get_string(f):
	length = f.read(4)
	assert len(length) == 4
	length, = struct.unpack("<I", length)
	# Sanity check the value.
	if length > 1000000:
		raise ValueError("Length %i way too long on received string! Framing error?" % length)
	datum = f.read(length)
	assert len(datum) == length
	return datum

def send_string(f, s):
	with global_data_stats_lock:
		global_stats["data_sent"] += 4 + len(s)
	f.write(struct.pack("<I", len(s)) + s)
	f.flush()

def global_data_stats_thread():
	INTERVAL = 60.0
	data_sent = 0
	while True:
		time.sleep(INTERVAL)
		with global_data_stats_lock:
			previous, data_sent = data_sent, global_stats["data_sent"]
			print "Current rate: %.2f KiB/s" % ((data_sent - previous) / INTERVAL / 2**10.0,)

def broadcast_message(message_type, string):
	global global_player_handlers
	with global_lock:
		for handler in global_player_handlers:
			try:
				handler.wfile.write(message_type)
				send_string(handler.wfile, string)
			except (AttributeError, socket.error):
				print "Culling:", handler.entity_id
				handler.keep_going = False
		# Filter down to just the good handlers.
		global_player_handlers = [handler for handler in global_player_handlers if handler.keep_going]

class GameManager:
	max_ticks_per_second = 60

	def __init__(self, args):
		self.sim = simulation.GameSimulation(is_server_side=True)
		self.sim.difficulty = args.difficulty
		self.sim.debug = args.debug
		self.input_queue = Queue.Queue()
		self.tick_count = 0

		# This is where a game gets server-side initialized. (Cart created, terrain, etc.)
		simulation.initialize_game(self.sim)

	def introduce_player(self, data):
		with global_lock:
			player = self.sim.add_entity(simulation.Player, [(0, 0, -100), data["name"]])
			player.respawn()
		return player.ent_id

	def server_side_ticks_thread(self):
		begin_sleep_time = last_complete_update = link.gettimeofday_wrapper()

		while True:
			# Process player inputs.
			try:
				while True:
					inp = self.input_queue.get_nowait()
					if inp["player_ent_id"] not in self.sim.entities:
						print "Bad input for player:", inp["player_ent_id"]
						continue
					player_entity = self.sim.entities[inp["player_ent_id"]]
					player_entity.input_motion_vector = inp["motion_vector"]
					player_entity.facing = inp["facing"]
					player_entity.tilt = inp["tilt"]
					for command in inp["commands"]:
						player_entity.try_to_command(command)
#					print "Got input:", inp
			except Queue.Empty:
				pass

			# Perform a step, using link.gettimeofday_wrapper to get very precise time.
			now = link.gettimeofday_wrapper()
			elapsed = float(now - begin_sleep_time)
#			if elapsed > 2.0 * (1.0 / self.max_ticks_per_second):
#				print "Excessive elapsed time:", elapsed
			self.sim.step(min(0.1, elapsed))	

			# Begin rare processing.
			if self.tick_count % SERVER_STATE_FRAMES == 0:
				cart = self.sim.get_ent(lambda x: isinstance(x, simulation.Cart)) 
#				print cart.push_state, cart.get_xyz()
				# Cull players who have quit.
				with global_lock:
					valid_player_ent_ids = set(handler.entity_id for handler in global_player_handlers)
					for ent_id, ent in self.sim.entities.items():
						if isinstance(ent, simulation.Player) and ent_id not in valid_player_ent_ids:
							self.sim.remove_entity(ent_id)
					# Every so often broadcast the entire game state.
					sim_state = self.sim.serialize_game_state()
				broadcast_message("s", sim_state)
			elif self.tick_count % SERVER_PATCHES_FRAMES == 0:
				# Pretty much every tick transmit patches.
				with global_lock:
					sim_state = self.sim.serialize_game_patches()
				broadcast_message("e", sim_state)

			begin_sleep_time = link.gettimeofday_wrapper()
			target_wakeup_time = last_complete_update + fractions.Fraction(1, self.max_ticks_per_second)
			sleep_time = target_wakeup_time - begin_sleep_time
			if sleep_time > 0:
				time.sleep(float(sleep_time))
			else:
				print "Missed frame deadline:", sleep_time, float(sleep_time)
			last_complete_update = link.gettimeofday_wrapper()
			wakeup_time_error = last_complete_update - target_wakeup_time
			# Print a warning if there is a large wakeup time error.
#			if wakeup_time_error > 0.5 * (1.0 / self.max_ticks_per_second):
#				print "Wakeup time error:", wakeup_time_error, float(wakeup_time_error)
			self.tick_count += 1

class Handler(SocketServer.StreamRequestHandler):
	entity_id = None

	def handle(self):
		print "Connection from:", self.client_address[0]
		self.keep_going = True

		with global_lock:
			global_player_handlers.append(self)

		while self.keep_going:
			# Get one byte of command.
			command = self.rfile.read(1)

			# Synchronous query message type.
			if command == "q":
				payload = json.loads(get_string(self.rfile))
				print "Query: %r" % (payload,)
				# Set a default response.
				response = "BAD_QUERY"
				if isinstance(payload, dict) and "type" in payload:
					query_type = payload["type"]
					if query_type == "ping":
						response = "pong"
					elif query_type == "introduce_player":
						self.entity_id = game.introduce_player(payload)
						print "Setting connection player ent_id:", self.entity_id
						response = self.entity_id
				self.wfile.write("r")
				send_string(self.wfile, json.dumps(response))

			# Player input message type.
			elif command == "i":
				payload = json.loads(get_string(self.rfile))
				payload["player_ent_id"] = self.entity_id
				game.input_queue.put(payload)

			# Exit.
			elif command == "x":
				print "Cleanly exiting:", self.entity_id
				with global_lock:
					if self in global_player_handlers:
						global_player_handlers.remove(self)
				self.keep_going = False

if __name__ == "__main__":
	import argparse
	parser = argparse.ArgumentParser(prog="server.py", description="cart-pusher")
	parser.add_argument("--difficulty", type=int, default=10, help="Difficulty of the game.")
	parser.add_argument("--debug", action="store_const", const=True, default=False, help="Turn on debug mobility.")
	args = parser.parse_args()

	print "=== cart-pusher game server ==="
	print "Running on port %s" % PORT
	game = GameManager(args)
	# Launch a thread that continuously runs the game simulation.
	thread.start_new_thread(global_data_stats_thread, ())
	thread.start_new_thread(game.server_side_ticks_thread, ())
	Server(("", PORT), Handler).serve_forever()

