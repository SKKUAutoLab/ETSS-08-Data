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
import random
import time
import numpy as np

import cv2

try:
	import pygame
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_q
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# NOTE: Initiate display to show the result
NUMBER_IMAGE   = 2000  # Number of image to get bounding box
VIEW_WIDTH     = 1280  # Resolution width of dataset
VIEW_HEIGHT    = 720   # Resolution height of dataset
FPS            = 20    # Frame per second
NUM_SEC_GET    = 1     # Get annotation per second
SAVE_IMG       = False # is save bounding box

# NOTE: folder to store the result
FOLDER_IMG_RGB   = "tss_out/tss_out_rgb_img"
FOLDER_IMG_INS   = "tss_out/tss_out_ins_img"
FOLDER_ANNO_BBOX = "tss_out/tss_out_bbox"
FOLDER_ANNO_INS  = "tss_out/tss_out_ins"

class CustomTimer:
	def __init__(self):
		try:
			self.timer = time.perf_counter
		except AttributeError:
			self.timer = time.time

	def time(self):
		return self.timer()


class DisplayManager:
	def __init__(self, grid_size, window_size):
		pygame.init()
		pygame.font.init()
		self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

		self.grid_size   = grid_size
		self.window_size = window_size
		self.sensor_list = []

	def get_window_size(self):
		return [int(self.window_size[0]), int(self.window_size[1])]

	def get_display_size(self):
		return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

	def get_display_offset(self, gridPos):
		dis_size = self.get_display_size()
		return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

	def add_sensor(self, sensor):
		self.sensor_list.append(sensor)

	def get_sensor_list(self):
		return self.sensor_list

	def render(self):
		if not self.render_enabled():
			return

		for s in self.sensor_list:
			s.render()

		pygame.display.flip()

	def destroy(self):
		for s in self.sensor_list:
			s.destroy()

	def render_enabled(self):
		return self.display != None


class SensorManager:
	def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
		self.surface        = None
		self.world          = world
		self.display_man    = display_man
		self.display_pos    = display_pos
		self.sensor         = self.init_sensor(sensor_type, transform, attached, sensor_options)
		self.sensor_options = sensor_options
		self.timer          = CustomTimer()

		self.time_processing = 0.0
		self.tics_processing = 0

		self.display_man.add_sensor(self)

		self.current_num_img = 0  # number of image have been saved

	def init_sensor(self, sensor_type, transform, attached, sensor_options):
		self.disp_size = self.display_man.get_display_size()
		if sensor_type == 'RGBCamera':
			camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
			# camera_bp.set_attribute('image_size_x', str(self.disp_size[0]))  # Get the size based on the grid
			# camera_bp.set_attribute('image_size_y', str(self.disp_size[1]))  # Get the size based on the grid

			camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
			camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))

			for key in sensor_options:
				camera_bp.set_attribute(key, sensor_options[key])

			camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
			camera.listen(self.save_rgb_image)

			return camera

		elif sensor_type == 'DepthCamera':
			depth_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
			depth_bp.set_attribute('image_size_x', str(self.disp_size[0]))
			depth_bp.set_attribute('image_size_y', str(self.disp_size[1]))

			for key in sensor_options:
				depth_bp.set_attribute(key, sensor_options[key])

			depth = self.world.spawn_actor(depth_bp, transform, attach_to=attached)
			depth.listen(self.save_depth_image)

			return depth

		elif sensor_type == 'InstanceSegmentationCamera':
			insseg_bp = self.world.get_blueprint_library().find('sensor.camera.instance_segmentation')
			# insseg_bp.set_attribute('image_size_x', str(self.disp_size[0]))
			# insseg_bp.set_attribute('image_size_y', str(self.disp_size[1]))

			insseg_bp.set_attribute('image_size_x', str(self.display_man.window_size[0]))
			insseg_bp.set_attribute('image_size_y', str(self.display_man.window_size[1]))

			for key in sensor_options:
				insseg_bp.set_attribute(key, sensor_options[key])

			insseg = self.world.spawn_actor(insseg_bp, transform, attach_to=attached)
			insseg.listen(self.save_instance_segmentation_image)

			return insseg

		elif sensor_type == 'SemanticSegmentationCamera':
			semseg_bp = self.world.get_blueprint_library().find('sensor.camera.instance_segmentation')
			semseg_bp.set_attribute('image_size_x', str(self.disp_size[0]))
			semseg_bp.set_attribute('image_size_y', str(self.disp_size[1]))

			for key in sensor_options:
				semseg_bp.set_attribute(key, sensor_options[key])

			semseg = self.world.spawn_actor(semseg_bp, transform, attach_to=attached)
			semseg.listen(self.save_semantic_segmentation_image)

			return semseg

		elif sensor_type == 'SemanticLiDAR':
			lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
			lidar_bp.set_attribute('range', '100')

			for key in sensor_options:
				lidar_bp.set_attribute(key, sensor_options[key])

			lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

			lidar.listen(self.save_semanticlidar_image)

			return lidar

		elif sensor_type == 'LiDAR':
			lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
			lidar_bp.set_attribute('range', '100')
			lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
			lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
			lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

			for key in sensor_options:
				lidar_bp.set_attribute(key, sensor_options[key])

			lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

			lidar.listen(self.save_lidar_image)

			return lidar

		else:
			return None

	def get_sensor(self):
		return self.sensor

	def save_semantic_segmentation_image(self, image):
		t_start = self.timer.time()

		image.convert(carla.ColorConverter.CityScapesPalette)
		array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
		array = np.reshape(array, (image.height, image.width, 4))
		array = array[:, :, :3]
		array = array[:, :, ::-1]

		if self.display_man.render_enabled():
			self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

		t_end = self.timer.time()
		self.time_processing += (t_end - t_start)
		self.tics_processing += 1

	def save_instance_segmentation_image(self, image):
		t_start = self.timer.time()

		image.convert(carla.ColorConverter.Raw)
		array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
		array = np.reshape(array, (image.height, image.width, 4))

		# NOTE: save image
		if SAVE_IMG:
			if self.current_num_img < NUMBER_IMAGE:
				if self.tics_processing % (FPS // NUM_SEC_GET) == 0:
					self.current_num_img = self.current_num_img + 1
					cv2.imwrite(f"{FOLDER_IMG_INS}/{self.tics_processing:08d}.png", array)

		array = array[:, :, :3]
		array = array[:, :, ::-1]

		# NOTE: resize array for display
		array = cv2.resize(array, (self.disp_size[0], self.disp_size[1]), interpolation=cv2.INTER_AREA)

		if self.display_man.render_enabled():
			self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

		t_end = self.timer.time()
		self.time_processing += (t_end - t_start)
		self.tics_processing += 1

	def save_depth_image(self, image):
		t_start = self.timer.time()

		image.convert(carla.ColorConverter.LogarithmicDepth)
		array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
		array = np.reshape(array, (image.height, image.width, 4))
		array = array[:, :, :3]
		array = array[:, :, ::-1]

		if self.display_man.render_enabled():
			self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

		t_end = self.timer.time()
		self.time_processing += (t_end - t_start)
		self.tics_processing += 1

	def save_rgb_image(self, image):
		t_start = self.timer.time()

		image.convert(carla.ColorConverter.Raw)
		array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
		array = np.reshape(array, (image.height, image.width, 4))

		# NOTE: save image
		if SAVE_IMG:
			if self.current_num_img < NUMBER_IMAGE:
				if self.tics_processing % (FPS // NUM_SEC_GET) == 0:
					self.current_num_img = self.current_num_img + 1
					cv2.imwrite(f"{FOLDER_IMG_RGB}/{self.tics_processing:08d}.jpg", array)
			else:
				print("FINISH CAPTURING")

		array = array[:, :, :3]
		array = array[:, :, ::-1]

		# NOTE: resize array for display
		array = cv2.resize(array, (self.disp_size[0], self.disp_size[1]), interpolation=cv2.INTER_AREA)

		if self.display_man.render_enabled():
			self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

		t_end = self.timer.time()
		self.time_processing += (t_end-t_start)
		self.tics_processing += 1

	def save_lidar_image(self, image):
		t_start = self.timer.time()

		disp_size = self.display_man.get_display_size()
		lidar_range = 2.0*float(self.sensor_options['range'])

		points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
		points = np.reshape(points, (int(points.shape[0] / 4), 4))
		lidar_data = np.array(points[:, :2])
		lidar_data *= min(disp_size) / lidar_range
		lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
		lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
		lidar_data = lidar_data.astype(np.int32)
		lidar_data = np.reshape(lidar_data, (-1, 2))
		lidar_img_size = (disp_size[0], disp_size[1], 3)
		lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

		lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

		if self.display_man.render_enabled():
			self.surface = pygame.surfarray.make_surface(lidar_img)

		# DEBUG:
		# if self.tics_processing % 30 == 0:
		# 	print(f"{self.tics_processing}")

		t_end = self.timer.time()
		self.time_processing += (t_end-t_start)
		self.tics_processing += 1

	def save_semanticlidar_image(self, image):
		t_start = self.timer.time()

		disp_size = self.display_man.get_display_size()
		lidar_range = 2.0*float(self.sensor_options['range'])

		points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
		points = np.reshape(points, (int(points.shape[0] / 6), 6))
		lidar_data = np.array(points[:, :2])
		lidar_data *= min(disp_size) / lidar_range
		lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
		lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
		lidar_data = lidar_data.astype(np.int32)
		lidar_data = np.reshape(lidar_data, (-1, 2))
		lidar_img_size = (disp_size[0], disp_size[1], 3)
		lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

		lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

		if self.display_man.render_enabled():
			self.surface = pygame.surfarray.make_surface(lidar_img)

		t_end = self.timer.time()
		self.time_processing += (t_end-t_start)
		self.tics_processing += 1

	def save_radar_image(self, radar_data):
		t_start = self.timer.time()
		points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
		points = np.reshape(points, (len(radar_data), 4))

		t_end = self.timer.time()
		self.time_processing += (t_end-t_start)
		self.tics_processing += 1

	def render(self):
		if self.surface is not None:
			offset = self.display_man.get_display_offset(self.display_pos)
			self.display_man.display.blit(self.surface, offset)

	def destroy(self):
		self.sensor.destroy()


def create_list_location_on_town():
	# NOTE: Town10HD
	list_location = [
		carla.Transform(
			carla.Location(x=-53.3, y=145.8, z=0.600000),
			carla.Rotation(pitch=0.000000, yaw=-45.0, roll=0.000000)
		),
		carla.Transform(
			carla.Location(x=-58.7, y=36.2, z=0.600000),
			carla.Rotation(pitch=0.000000, yaw=-45.0, roll=0.000000)
		)
	]
	return list_location


def run_simulation(args, client):
	"""
	This function performed one test run using the args parameters
	and connecting to the carla client passed.
	"""

	# NOTE: location and position for capture the dataset
	list_location   = create_list_location_on_town()
	camera_position = carla.Transform(carla.Location(x=0, y=5, z=15), carla.Rotation(pitch=-45.0, yaw=-45))

	display_manager = None
	vehicle = None
	vehicle_list = []
	timer = CustomTimer()

	try:

		# NOTE: Getting the world and
		world = client.get_world()
		original_settings = world.get_settings()

		if args.sync:
			traffic_manager = client.get_trafficmanager(8000)
			settings = world.get_settings()
			traffic_manager.set_synchronous_mode(True)
			settings.synchronous_mode = True
			settings.fixed_delta_seconds = 1 / FPS
			world.apply_settings(settings)


		# NOTE: Instanciating the vehicle to which we attached the sensors
		bp = world.get_blueprint_library().filter('vehicle.bh.crossbike')[0]  # Set vehicle
		# vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))  # Set random position on map
		vehicle = world.spawn_actor(bp, list_location[0])
		vehicle_list.append(vehicle)
		# vehicle.set_autopilot(True)  # Set autorun by AI

		# NOTE: Display Manager organize all the sensors an its display in a window
		# If can easily configure the grid and the total window size
		display_manager = DisplayManager(grid_size=[1, 2], window_size=[args.width, args.height])

		# NOTE: setup all the sensors from the vehicle
		# Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
		# and assign each of them to a grid position,

		SensorManager(
			world,
			display_manager,
			'RGBCamera',
			camera_position,
			vehicle,
			{},
			display_pos=[0, 0]
		)
		SensorManager(
			world,
			display_manager,
			'InstanceSegmentationCamera',
			camera_position,
			vehicle,
			{},
			display_pos=[0, 1]
		)

		# NOTE: Simulation loop
		call_exit = False
		time_init_sim = timer.time()
		while True:
			# Carla Tick
			if args.sync:
				world.tick()
			else:
				world.wait_for_tick()

			# Render received data
			display_manager.render()

			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					call_exit = True
				elif event.type == pygame.KEYDOWN:
					if event.key == K_ESCAPE or event.key == K_q:
						call_exit = True
						break

			if call_exit:
				break

	finally:
		if display_manager:
			display_manager.destroy()

		client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

		world.apply_settings(original_settings)


def make_dir(path):
	"""
	Create folder (recursive)

	Args:
		path (str): path of folder which is need to be created
	"""
	try:
		if not os.path.exists(path):
			os.makedirs(path)
	except FileExistsError:
		return


def main():
	argparser = argparse.ArgumentParser(
		description='CARLA Sensor tutorial')
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
		'--sync',
		action='store_true',
		help='Synchronous mode execution')
	argparser.add_argument(
		'--async',
		dest='sync',
		action='store_false',
		help='Asynchronous mode execution')
	argparser.set_defaults(sync=True)

	args = argparser.parse_args()

	args.width  = int(VIEW_WIDTH)
	args.height = int(VIEW_HEIGHT)

	# NOTE: create the folder for store dataset
	make_dir(FOLDER_IMG_RGB)
	make_dir(FOLDER_IMG_INS)
	make_dir(FOLDER_ANNO_BBOX)
	make_dir(FOLDER_ANNO_INS)

	# NOTE: run the capture
	try:
		client = carla.Client(args.host, args.port)
		client.set_timeout(5.0)

		run_simulation(args, client)

	except KeyboardInterrupt:
		print('\nCancelled by user. Bye!')


if __name__ == '__main__':
	main()
