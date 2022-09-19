#!/usr/bin/env python

import sys
import os
import glob

import cv2

from tqdm import tqdm


def draw_instance(img_rgb_path, img_ins_path):
	img_rgb = cv2.imread(img_rgb_path)
	img_ins = cv2.imread(img_ins_path, cv2.IMREAD_UNCHANGED)  # IMREAD_UNCHANGED => open image with the alpha channel

	height, width = img_ins.shape[:2]
	for y in range(height):
		for x in range(width):
			# print(img_ins[y, x, 2])
			# print(img_ins[y, x])

			if img_ins[y, x, 2] in [4, 10]:  # 4:pedestrian, 10:vehicles
				overlay_color = img_ins[y, x, :3]  # first three elements are color (RGB)
				overlay_alpha = 0.5  # 4th element is the alpha channel, convert from 0-255 to 0.0-1.0

				# get the color from the background image
				background_color = img_rgb[y, x]

				# combine the background color and the overlay color weighted by alpha
				composite_color = background_color * (1 - overlay_alpha) + overlay_color * overlay_alpha

				# update the background image in place
				img_rgb[y, x] = composite_color

	return img_rgb

	# cv2.imshow(f"{os.path.basename(img_rgb_path)}", img_rgb)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

def main():
	# This is needed to avoid strange crashes related to opencv
	cv2.setNumThreads(0)

	folder_img_rbg   = "tss_out/tss_out_rgb_img"
	folder_img_ins   = "tss_out/tss_out_ins_img"
	folder_img_drawn = "tss_out/tss_out_drawn"

	img_rgb_list   = glob.glob(os.path.join(folder_img_rbg, "*.jpg"))

	for img_rgb_path in tqdm(img_rgb_list):
		basename       = os.path.basename(img_rgb_path)
		basename_noext = os.path.splitext(basename)[0]
		img_ins_path   = os.path.join(folder_img_ins, f"{basename_noext}.png")

		if os.path.exists(img_ins_path):
			img_rgb = draw_instance(img_rgb_path, img_ins_path)
			cv2.imwrite(os.path.join(folder_img_drawn, f"{basename_noext}.jpg"), img_rgb)

		# break


if __name__ == '__main__':
	main()
