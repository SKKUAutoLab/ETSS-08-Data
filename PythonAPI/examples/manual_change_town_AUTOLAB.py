#!/usr/bin/env python

import glob
import os
import sys

try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass

import carla

import argparse
import math


def main():
	argparser = argparse.ArgumentParser(
		description=__doc__)
	argparser.add_argument(
		'--host',
		metavar='H',
		default='127.0.0.1',
		help='IP of the host server (default: 127.0.0.1)')
	argparser.add_argument(
		'-p', '--port',
		metavar='P',
		default=2000,
		type=int,
		help='TCP port to listen to (default: 2000)')
	argparser.add_argument(
		'-s', '--speed',
		metavar='FACTOR',
		default=1.0,
		type=float,
		help='rate at which the weather changes (default: 1.0)')
	args = argparser.parse_args()

	speed_factor = args.speed
	update_freq = 0.1 / speed_factor

	client = carla.Client(args.host, args.port)
	client.set_timeout(2.0)
	world = client.get_world()
	world = client.load_world('Town02')



if __name__ == '__main__':

	main()
