#!/usr/bin/env python

import sys
import os
import glob
import json
import threading
from multiprocessing import Process

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError
import cv2

from tqdm import tqdm

location          = "tss_out"
folder_img_rbg    = f"{location}/tss_out_rgb_img"
folder_img_ins    = f"{location}/tss_out_ins_img"
folder_anno_ins   = f"{location}/tss_out_ins"
folder_anno_bbox  = f"{location}/tss_out_bbox"
number_of_process = 4

annotation_class = {
	'vehicle'   : 10,
	'pedestrian': 4
}


class NumpyEncoder(json.JSONEncoder):
	def default(self, obj):
		if isinstance(obj, np.ndarray):
			return obj.tolist()
		return json.JSONEncoder.default(self, obj)


def create_annotations(segs):
	inses  = []
	bboxes = []
	ids    = []
	for key, value in segs.items():
		if len(value["segment"]) > 10:   # The minimum number of point for segmentation
			arrary_value = np.array(value["segment"])
			id_un = value["class_id"]
			ins   = []

			try:
				hull  = ConvexHull(arrary_value)
			except:
				continue


			for points in hull.points:
				ins.append(points)
			ins = np.array(ins)

			x_min = min(ins[:, 0])
			y_min = min(ins[:, 1])
			x_max = max(ins[:, 0])
			y_max = max(ins[:, 1])

			inses.append(ins)
			bboxes.append([x_min, y_min, x_max, y_max])
			ids.append(id_un)
	return ids, inses, bboxes


def compare_color(pixel_1, pixel_2):
	if len(pixel_1) != len(pixel_2):
		return False

	for co_1, co_2 in zip(pixel_1, pixel_2):
		if co_1 != co_2:
			return False

	return True


def extract_instance(img_rgb_path, img_ins_path):
	img_rgb = cv2.imread(img_rgb_path)
	img_ins = cv2.imread(img_ins_path, cv2.IMREAD_UNCHANGED)  # IMREAD_UNCHANGED => open image with the alpha channel

	segs = {}

	height, width = img_ins.shape[:2]
	for y in range(height):
		for x in range(width):

			# NOTE: processing each ID after finding one
			if img_ins[y, x, 2] in [4, 10]:
				key_seg = f"{img_ins[y, x, 0]}_{img_ins[y, x, 1]}"

				# Create the new one if it is not exist
				if key_seg not in segs:
					segs[key_seg] = {
						"class_id" : img_ins[y, x, 2],
						"segment"  : [],
					}

				segs[key_seg]["segment"].append([x, y])

	# DEBUG:
	# print(segs)

	return segs


def divide_chunks(l, n):
	# looping till length l
	for i in range(0, len(l), n):
		yield l[i:i + n]


def extract_instance_segmentation(img_rgb_list):

	for img_rgb_path in tqdm(img_rgb_list, desc=f""):
		basename        = os.path.basename(img_rgb_path)
		basename_noext  = os.path.splitext(basename)[0]
		img_ins_path    = os.path.join(folder_img_ins, f"{basename_noext}.png")
		anno_ins_path   = os.path.join(folder_anno_ins, f"{basename_noext}.txt")
		anno_bbox_path  = os.path.join(folder_anno_bbox, f"{basename_noext}.txt")

		if os.path.exists(img_ins_path):
			segs                = extract_instance(img_rgb_path, img_ins_path)
			ids, inses, bboxes  = create_annotations(segs)

			# NOTE: output instance annotations
			with open(anno_ins_path, "w") as f:
				for anno, id_un in zip(inses, ids):
					f.write(f"{id_un} ")
					for point in anno:
						f.write(f"{int(point[0])} {int(point[1])} ")
					f.write("\n")

			# NOTE: output bounding box annotations
			with open(anno_bbox_path, "w") as f:
				for anno, id_un in zip(bboxes, ids):
					f.write(f"{id_un} ")
					f.write(f"{int(anno[0])} {int(anno[1])} {int(anno[2])} {int(anno[3])}\n")

		# DEBUG:
		# break


def main():
	# This is needed to avoid strange crashes related to opencv
	cv2.setNumThreads(0)

	img_rgb_list = glob.glob(os.path.join(folder_img_rbg, "*.jpg"))
	img_rgb_lists = list(divide_chunks(img_rgb_list, len(img_rgb_list) // number_of_process))

	# NOTE: Define threads
	# threads = []
	# for rgb_list in img_rgb_lists:
	# 	threads.append(threading.Thread(target=extract_instance_segmentation, args=(rgb_list,)))
	#
	# # NOTE: Start threads
	# for thread in threads:
	# 	thread.start()
	#
	# # NOTE: Wait all threads stop
	# for thread in threads:
	# 	thread.join()

	# NOTE: Define processes
	processes = []
	for rgb_list in img_rgb_lists:
		processes.append(Process(target=extract_instance_segmentation, args=(rgb_list,)))

	# NOTE: Start processes
	print(f"{len(processes)} processes are running")
	for process in processes:
		process.start()

	# NOTE: Wait all processes stop
	for process in processes:
		process.join()

	print(f"{len(processes)} processes finished")


if __name__ == "__main__":
	main()
