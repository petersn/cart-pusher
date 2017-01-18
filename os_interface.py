#! /usr/bin/python

import time, functools, platform

unit_table = {"kb": 2**10, "mb": 2**20}
start_time = time.time()

def cache(function):
	cache = [0, None]
	@functools.wraps(function)
	def f():
		if time.time() < cache[0] + 1:
			return cache[1]
		cache[0] = time.time()
		cache[1] = function()
		return cache[1]
	return f

@cache
def linux_memory_usage():
	try:
		for line in open("/proc/self/status"):
			if line.startswith("VmRSS:"):
				number, unit = line.split(":")[1].strip().split()
				return int(number) * unit_table[unit.lower()]
	except:
		return 0

last_sample_time = time.time()
last_sample_usage = 0.0
@cache
def linux_cpu_usage():
	global last_sample_time, last_sample_usage
	ru = resource.getrusage(resource.RUSAGE_SELF)
	sample_usage = ru.ru_utime + ru.ru_stime
	now = time.time()
	utilization = (sample_usage - last_sample_usage) / (now - last_sample_time)
	last_sample_time = now
	last_sample_usage = sample_usage
	return utilization

@cache
def windows_memory_usage():
	return 0

@cache
def windows_cpu_usage():
	return 0

# Set default implementations that always return zero.
memory_usage = cpu_usage = lambda: 0

system = platform.system()
if system == "Linux":
	import resource
	memory_usage = linux_memory_usage
	cpu_usage = linux_cpu_usage
elif system == "Windows":
	memory_usage = windows_memory_usage
	cpu_usage = windows_cpu_usage
else:
	print "Warning: Unrecognized system %r. Some debugging information unavailable." % (system,)

