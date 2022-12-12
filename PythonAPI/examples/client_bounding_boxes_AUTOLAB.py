#!/usr/bin/env python
# ==================================================================== #
# Copyright (C) 2022 - Automation Lab - Sungkyunkwan University
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
# ==================================================================== #

from __future__ import annotations
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


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

import weakref
import random

try:
	import pygame
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_SPACE
	from pygame.locals import K_a
	from pygame.locals import K_d
	from pygame.locals import K_s
	from pygame.locals import K_w
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
	import numpy as np
except ImportError:
	raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# NOTE: Inite display to show the result
NUMBER_IMAGE = 2000  # Number of image to get bounding box
VIEW_WIDTH   = 1280  # Resolution width of dataset
VIEW_HEIGHT  = 720   # Resolution height of dataset
VIEW_FOV     = 90	 # Field of view
FPS          = 20	 # Frame per second
NUM_SEC_GET  = 1	 # Get bounding box per second
SAVE_BBOX    = True  # is save bounding box

actor_annotations = {
	"reference": {
		"passenger_car": 0,
		"truck": 1,
		"motorcycle": 2,
		"bicycle": 3,
		"escooter": 5,
		"others":9
	},
	"classification": {
		"vehicle.vespa.zx125": 2,
		"vehicle.mini.cooper_s": 0,
		"vehicle.mini.cooper_s_2020": 0,
		"vehicle.mini.cooper_s_2021": 0,
		"vehicle.micro.microlino": 0,
		"vehicle.ford.ambulance": 1,
		"vehicle.ford.crown": 0,
		"vehicle.ford.mustang": 0,
		"vehicle.citroen.c3": 0,
		"vehicle.chevrolet.impala": 0,
		"vehicle.audi.a2": 0,
		"vehicle.nissan.micra": 0,
		"vehicle.carlamotors.carlacola": 1,
		"vehicle.carlamotors.firetruck": 1,
		"vehicle.audi.tt": 0,
		"vehicle.bmw.grandtourer": 0,
		"vehicle.harley-davidson.low_rider": 2,
		"vehicle.bmw.isetta": 0,
		"vehicle.dodge.charger_2020": 0,
		"vehicle.dodge.charger_police": 0,
		"vehicle.dodge.charger_police_2020": 0,
		"vehicle.jeep.wrangler_rubicon": 0,
		"vehicle.mercedes.coupe": 0,
		"vehicle.mercedes.coupe_2020": 0,
		"vehicle.mercedes.sprinter": 0,
		"vehicle.mini.cooperst": 0,
		"vehicle.nissan.patrol": 0,
		"vehicle.nissan.patrol_2021": 0,
		"vehicle.seat.leon": 0,
		"vehicle.toyota.prius": 0,
		"vehicle.yamaha.yzf": 2,
		"vehicle.kawasaki.ninja": 2,
		"vehicle.bh.crossbike": 3,
		"vehicle.tesla.model3": 0,
		"vehicle.gazelle.omafiets": 3,
		"vehicle.tesla.cybertruck": 1,
		"vehicle.diamondback.century": 3,
		"vehicle.audi.etron": 0,
		"vehicle.volkswagen.t2": 0,
		"vehicle.volkswagen.t2_2021": 0,
		"vehicle.lincoln.mkz_2017": 0,
		"vehicle.lincoln.mkz_2020": 0,
		"vehicle.mustang.mustang": 0,
		"vehicle.escooter1_make.kanglim_escooter": 5,
		"vehicle.escooter2_make.girl_escooter": 5,
		"vehicle.escooter3_make.boy_escooter": 5,
		"vehicle.escooter4_make.glassesboy_escooter": 5,
		"vehicle.escooter5_make.blond_escooter": 5,
		"vehicle.escooter6_make.me_escooter": 5,
		"vehicle.escooter7_make.tallwoman_escooter": 5,
		"vehicle.escooter8_make.tallboy_escooter": 5,
		"vehicle.escooter9_make.trans_escooter": 5,
		"vehicle.escooter10_make.revo_escooter": 5
	}
}


# NOTE: color of bounding box
BB3D_COLOR  = (248, 64, 24)
BB2D_COLOR  = (50, 168, 82)

# ==============================================================================
# -- ClientSideBoundingBoxes ---------------------------------------------------
# ==============================================================================


class ClientSideBoundingBoxes(object):
	"""
	This is a module responsible for creating 3D bounding boxes and drawing them
	client-side on pygame surface.
	"""

	@staticmethod
	def filter_bounding_boxes(bounding_3D_boxes_temp, bounding_2D_boxes_temp, bounding_box_actor_temp):
		"""
		Filter the wrong bounding box,
			which has the x_max == 0, y_max == 0 in 2D bounding box
		"""

		bounding_2D_boxes  = []
		bounding_3D_boxes  = []
		bounding_box_actor = []

		for bbox_2D, bbox_3D, actor in zip(bounding_2D_boxes_temp, bounding_3D_boxes_temp, bounding_box_actor_temp):
			# DEBUG:
			# print(dir(actor.attributes))
			# print(actor.attributes)
			# sys.exit()
			
			# bbox_2D == [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min]]
			if "number_of_wheels" in actor.attributes and actor.attributes["number_of_wheels"] != 2:  # if the vehicle has more than 2 wheels
				if abs(bbox_2D[2][0] - bbox_2D[0][0]) < max(VIEW_WIDTH, VIEW_HEIGHT) // 64  \
					or abs(bbox_2D[2][1] - bbox_2D[0][1]) < max(VIEW_WIDTH, VIEW_HEIGHT) // 64:
					continue
			else:  # if the vehicle is 2 wheels or the pedestrian
				if abs(bbox_2D[2][0] - bbox_2D[0][0]) < max(VIEW_WIDTH, VIEW_HEIGHT) // 128  \
					or abs(bbox_2D[2][1] - bbox_2D[0][1]) < max(VIEW_WIDTH, VIEW_HEIGHT) // 100:
					continue

			if bbox_2D[2][0] > 0 and bbox_2D[2][1] > 0 and \
				bbox_2D[0][0] < VIEW_WIDTH and bbox_2D[0][1] < VIEW_HEIGHT:
				bounding_2D_boxes.append(bbox_2D)
				bounding_3D_boxes.append(bbox_3D)
				bounding_box_actor.append(actor)

		return 	bounding_3D_boxes, bounding_2D_boxes, bounding_box_actor	

	@staticmethod
	def get_bounding_boxes(actors, camera):
		"""
		Creates 3D, 2D bounding boxes based on carla actor list and camera.
		"""
		# NOTE: get the 3D bounding box
		bounding_3D_boxes  = None 
		bounding_box_actor = None
		for i in range(len(actors)): 
			for actor in actors[i]:
				bounding_3D_boxes_temp  = [ClientSideBoundingBoxes.get_bounding_box(actor, camera)]
				bounding_box_actor_temp = [actor for _ in range(len(bounding_3D_boxes_temp))]

				if bounding_3D_boxes is None:
					bounding_3D_boxes  = bounding_3D_boxes_temp
					bounding_box_actor = bounding_box_actor_temp
				else:
					bounding_3D_boxes  = bounding_3D_boxes + bounding_3D_boxes_temp
					bounding_box_actor = bounding_box_actor + bounding_box_actor_temp

		# NOTE: filter objects behind camera
		bounding_box_actor = [actor for bb, actor in zip(bounding_3D_boxes, bounding_box_actor) if all(bb[:, 2] > 0)]
		bounding_3D_boxes  = [bb for bb in bounding_3D_boxes if all(bb[:, 2] > 0)]

		# NOTE: get the 2D bounding box from 3D bounding box
		bounding_2D_boxes = []
		for bbox in bounding_3D_boxes:
			x_min = VIEW_WIDTH
			y_min = VIEW_HEIGHT
			x_max = 0
			y_max = 0
			for i in range(len(bbox)):
				x_min = min(x_min, int(bbox[i, 0]))
				y_min = min(y_min, int(bbox[i, 1]))
				x_max = max(x_max, int(bbox[i, 0]))
				y_max = max(y_max, int(bbox[i, 1]))
			bounding_2D_boxes.append([[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min]])
		
		return bounding_3D_boxes, bounding_2D_boxes, bounding_box_actor

	@staticmethod
	def draw_bounding_boxes(display, bounding_3D_boxes, bounding_2D_boxes):
		"""
		Draws bounding boxes on pygame display.
		"""

		bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
		bb_surface.set_colorkey((0, 0, 0))

		# 3D bounding box
		for bbox in bounding_3D_boxes:
			points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
			
			# draw lines
			# base
			pygame.draw.line(bb_surface, BB3D_COLOR, points[0], points[1])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[0], points[1])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[1], points[2])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[2], points[3])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[3], points[0])
			# top
			pygame.draw.line(bb_surface, BB3D_COLOR, points[4], points[5])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[5], points[6])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[6], points[7])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[7], points[4])
			# base-top
			pygame.draw.line(bb_surface, BB3D_COLOR, points[0], points[4])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[1], points[5])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[2], points[6])
			pygame.draw.line(bb_surface, BB3D_COLOR, points[3], points[7])

		# 2D bounding box
		for point in bounding_2D_boxes:
			pygame.draw.line(bb_surface, BB2D_COLOR, point[0], point[1], 3)	
			pygame.draw.line(bb_surface, BB2D_COLOR, point[1], point[2], 3)
			pygame.draw.line(bb_surface, BB2D_COLOR, point[2], point[3], 3)
			pygame.draw.line(bb_surface, BB2D_COLOR, point[3], point[0], 3)

		display.blit(bb_surface, (0, 0))

	@staticmethod
	def get_bounding_box(vehicle, camera):
		"""
		Returns 3D bounding box for a vehicle based on camera view.
		"""

		bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
		cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
		cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
		bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
		camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
		return camera_bbox

	@staticmethod
	def _create_bb_points(vehicle):
		"""
		Returns 3D bounding box for a vehicle.
		"""

		cords = np.zeros((8, 4))
		extent = vehicle.bounding_box.extent
		cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
		cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
		cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
		cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
		cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
		cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
		cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
		cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
		return cords

	@staticmethod
	def _vehicle_to_sensor(cords, vehicle, sensor):
		"""
		Transforms coordinates of a vehicle bounding box to sensor.
		"""

		world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
		sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
		return sensor_cord

	@staticmethod
	def _vehicle_to_world(cords, vehicle):
		"""
		Transforms coordinates of a vehicle bounding box to world.
		"""

		bb_transform = carla.Transform(vehicle.bounding_box.location)
		bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
		vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(vehicle.get_transform())
		bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
		world_cords = np.dot(bb_world_matrix, np.transpose(cords))
		return world_cords

	@staticmethod
	def _world_to_sensor(cords, sensor):
		"""
		Transforms world coordinates to sensor.
		"""

		sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
		world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
		sensor_cords = np.dot(world_sensor_matrix, cords)
		return sensor_cords

	@staticmethod
	def get_matrix(transform):
		"""
		Creates matrix from carla transform.
		"""

		rotation = transform.rotation
		location = transform.location
		c_y = np.cos(np.radians(rotation.yaw))
		s_y = np.sin(np.radians(rotation.yaw))
		c_r = np.cos(np.radians(rotation.roll))
		s_r = np.sin(np.radians(rotation.roll))
		c_p = np.cos(np.radians(rotation.pitch))
		s_p = np.sin(np.radians(rotation.pitch))
		matrix = np.matrix(np.identity(4))
		matrix[0, 3] = location.x
		matrix[1, 3] = location.y
		matrix[2, 3] = location.z
		matrix[0, 0] = c_p * c_y
		matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
		matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
		matrix[1, 0] = s_y * c_p
		matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
		matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
		matrix[2, 0] = s_p
		matrix[2, 1] = -c_p * s_r
		matrix[2, 2] = c_p * c_r
		return matrix


# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
	"""
	Basic implementation of a synchronous client.
	"""

	def __init__(self):
		self.client = None
		self.world = None
		self.camera = None
		self.car = None

		self.display = None
		self.image = None
		self.capture = True

	def camera_blueprint(self):
		"""
		Returns camera blueprint.
		"""

		camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
		camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
		camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
		camera_bp.set_attribute('fov', str(VIEW_FOV))
		return camera_bp

	def set_synchronous_mode(self, synchronous_mode):
		"""
		Sets synchronous mode.
		"""

		settings = self.world.get_settings()
		# SUGAR:
		settings.fixed_delta_seconds = 1 / FPS
		
		settings.synchronous_mode = synchronous_mode
		self.world.apply_settings(settings)

	def setup_car(self):
		"""
		Spawns actor-vehicle to be controled.
		"""

		car_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
		location = random.choice(self.world.get_map().get_spawn_points())
		self.car = self.world.spawn_actor(car_bp, location)

		# SUGAR: add auto pilot
		self.car.set_autopilot(True)


	def setup_camera(self):
		"""
		Spawns actor-camera to be used to render view.
		Sets calibration for client-side boxes rendering.
		"""

		# NOTE: set camera position
		# camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))  # 3rd view
		camera_transform = carla.Transform(carla.Location(x=1.6, z=1.7)) #  1st view, view of driver
		self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
		weak_self = weakref.ref(self)
		self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

		calibration = np.identity(3)
		calibration[0, 2] = VIEW_WIDTH / 2.0
		calibration[1, 2] = VIEW_HEIGHT / 2.0
		calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
		self.camera.calibration = calibration

	def control(self, car):
		"""
		Applies control to main car based on pygame pressed keys.
		Will return True If ESCAPE is hit, otherwise False to end main loop.
		"""

		keys = pygame.key.get_pressed()
		if keys[K_ESCAPE]:
			return True

		control = car.get_control()
		control.throttle = 0
		if keys[K_w]:
			control.throttle = 1
			control.reverse = False
		elif keys[K_s]:
			control.throttle = 1
			control.reverse = True
		if keys[K_a]:
			control.steer = max(-1., min(control.steer - 0.05, 0))
		elif keys[K_d]:
			control.steer = min(1., max(control.steer + 0.05, 0))
		else:
			control.steer = 0
		control.hand_brake = keys[K_SPACE]

		car.apply_control(control)
		return False

	@staticmethod
	def set_image(weak_self, img):
		"""
		Sets image coming from camera sensor.
		The self.capture flag is a mean of synchronization - once the flag is
		set, next coming image will be stored.
		"""

		self = weak_self()
		if self.capture:
			self.image = img
			self.capture = False

	def render(self, display):
		"""
		Transforms image from camera sensor and blits it to main pygame display.
		"""

		if self.image is not None:
			array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
			array = np.reshape(array, (self.image.height, self.image.width, 4))
			array = array[:, :, :3]
			array = array[:, :, ::-1]
			surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
			display.blit(surface, (0, 0))

	# SUGAR:
	def save_annotation_image(self):
		"""
		Save bounding box, and image with bounding box.
		"""
		pygame.image.save(self.display, f"out_rgb_bbox/{self.image.frame:08d}.jpg")

		# NOTE: write 2D bounding box
		with open(f"out_bbox/{self.image.frame:08d}.txt", "w") as f_write:
			# NOTE: write for vehicle
			for point, actor in zip(self.bounding_2D_boxes, self.bounding_box_actor):
				has_key = False
				for key, value in actor_annotations['classification'].items():
					if key in actor.type_id:
						f_write.write(f"{value} ")
						has_key = True
						break
				if not has_key:
					f_write.write(f"0 ")
				f_write.write(f"{point[0][0]} {point[0][1]} {point[2][0]} {point[2][1]}\n")


	# SUGAR:
	def save_raw_image(self):
		"""
		Save raw image.
		"""
		pygame.image.save(self.display, f"out_rgb/{self.image.frame:08d}.jpg")


	def game_loop(self):
		"""
		Main program loop.
		"""

		try:
			pygame.init()

			self.client = carla.Client('127.0.0.1', 2000)
			self.client.set_timeout(2.0)
			self.world = self.client.get_world()

			self.setup_car()
			self.setup_camera()

			self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
			pygame_clock = pygame.time.Clock()

			self.set_synchronous_mode(True)

			# NOTE: the list of object we get here.
			vehicles	= self.world.get_actors().filter('vehicle.*')
			pedestrians = self.world.get_actors().filter('walker.pedestrian.*')
			
			currend_frame = 0
			if self.image is not None:
				currend_frame = self.image.frame


			is_running	   = True
			num_img		  = 0
			is_fullfill_save = False
			while is_running:
				is_fullfill_save = False
				self.world.tick()

				self.capture = True
				pygame_clock.tick_busy_loop(20)

				self.render(self.display)
				self.bounding_3D_boxes, self.bounding_2D_boxes, self.bounding_box_actor = ClientSideBoundingBoxes.get_bounding_boxes([vehicles, pedestrians], self.camera)
				self.bounding_3D_boxes, self.bounding_2D_boxes, self.bounding_box_actor = ClientSideBoundingBoxes.filter_bounding_boxes(self.bounding_3D_boxes, self.bounding_2D_boxes, self.bounding_box_actor)

				
				if SAVE_BBOX:
					if self.image.frame - currend_frame >= FPS * NUM_SEC_GET:
						if len(self.bounding_box_actor) > 0: 
							is_fullfill_save = True
							
				# SUGAR: save the raw image
				if is_fullfill_save:
					self.save_raw_image()

				ClientSideBoundingBoxes.draw_bounding_boxes(self.display, self.bounding_3D_boxes, self.bounding_2D_boxes)

				# SUGAR: save the bounding box and the drawn-image
				if is_fullfill_save:
					self.save_annotation_image()
					currend_frame = self.image.frame

					num_img = num_img + 1
					if num_img >= NUMBER_IMAGE:
						is_running = False

				pygame.display.flip()

				pygame.event.pump()
				if self.control(self.car):
					return

		finally:
			self.set_synchronous_mode(False)
			self.camera.destroy()
			self.car.destroy()
			pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

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
	"""
	Initializes the client-side bounding box demo.
	"""

	# NOTE: generate the output folder
	make_dir("out_bbox")
	make_dir("out_rgb")
	make_dir("out_rgb_bbox")

	try:
		client = BasicSynchronousClient()
		client.game_loop()
	finally:
		print('EXIT')


if __name__ == '__main__':
	main()
