#!/usr/bin/env python

import sys
import os
import glob
import json

import numpy as np
from scipy.spatial import ConvexHull
import cv2

from tqdm import tqdm

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
	annos = []
	ids   = []
	for key, value in segs.items():
		if len(value["segment"]) > 10:   # The minimum number of point for segmentation
			arrary_value = np.array(value["segment"])
			id_un = value["class_id"]
			anno  = []
			hull  = ConvexHull(arrary_value)
			for simplex in hull.simplices:
				anno.append(arrary_value[simplex, 0])
			annos.append(anno)
			ids.append(id_un)
	return annos, ids


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

				# Create the new one if it is not exist
				if f"{img_ins[y, x, 0]}_{img_ins[y, x, 1]}" not in segs:
					segs[f"{img_ins[y, x, 0]}_{img_ins[y, x, 1]}"] = {
						"class_id" : img_ins[y, x, 2],
						"segment"  : [],
					}

				segs[f"{img_ins[y, x, 0]}_{img_ins[y, x, 1]}"]["segment"].append([y, x])

	return segs


def main():
	# This is needed to avoid strange crashes related to opencv
	cv2.setNumThreads(0)

	folder_img_rbg  = "tss_out/tss_out_rgb_img"
	folder_img_ins  = "tss_out/tss_out_ins_img"
	folder_anno_ins = "tss_out/tss_out_ins"

	img_rgb_list = glob.glob(os.path.join(folder_img_rbg, "*.jpg"))

	for img_rgb_path in tqdm(img_rgb_list):
		basename       = os.path.basename(img_rgb_path)
		basename_noext = os.path.splitext(basename)[0]
		img_ins_path   = os.path.join(folder_img_ins, f"{basename_noext}.png")
		anno_ins_path  = os.path.join(folder_anno_ins, f"{basename_noext}.txt")

		if os.path.exists(img_ins_path):
			segs       = extract_instance(img_rgb_path, img_ins_path)
			annos, ids = create_annotations(segs)

			# NOTE: output annotations
			with open(anno_ins_path, "w") as f:
				for anno, id_un in zip(annos, ids):
					f.write(f"{id_un} ")
					for point in anno:
						f.write(f"{point[0]} {point[1]} ")
					f.write("\n")


if __name__ == "__main__":
	main()
