#!/usr/bin/env python

import sys
import os
import glob

import cv2

from tqdm import tqdm


def create_coco_annotation(anno):
	pass


def extract_instance(img_rgb_path, img_ins_path):
	img_rgb = cv2.imread(img_rgb_path)
	img_ins = cv2.imread(img_ins_path, cv2.IMREAD_UNCHANGED)  # IMREAD_UNCHANGED => open image with the alpha channel

	height, width = img_ins.shape[:2]
	for y in range(height):
		for x in range(width):
			pass

def main():
	# This is needed to avoid strange crashes related to opencv
	cv2.setNumThreads(0)

	folder_img_rbg = "tss_out/tss_out_rgb_img"
	folder_img_ins = "tss_out/tss_out_ins_img"

	img_rgb_list = glob.glob(os.path.join(folder_img_rbg, "*.jpg"))

	for img_rgb_path in tqdm(img_rgb_list):
		basename = os.path.basename(img_rgb_path)
		basename_noext = os.path.splitext(basename)[0]
		img_ins_path = os.path.join(folder_img_ins, f"{basename_noext}.png")

		if os.path.exists(img_ins_path):
			anno = extract_instance(img_rgb_path, img_ins_path)


if __name__ == "__main__":
	main()
