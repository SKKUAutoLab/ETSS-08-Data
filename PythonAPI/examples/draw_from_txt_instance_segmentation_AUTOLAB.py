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

import sys
import os
import glob

import numpy as np
import cv2

from tqdm import tqdm


def draw_instance(img_rgb_path, txt_ins_path):
	img_rgb = cv2.imread(img_rgb_path)

	# Initialize blank mask image of same dimensions for drawing the shapes
	shapes = np.zeros_like(img_rgb, np.uint8)

	with open(txt_ins_path, "r") as f_read:
		lines = f_read.readlines()
		for line in lines:
			words = line.replace("\n", "").replace("\r", "").split(" ")
			pts   = []

			for index in range(1, len(words), 2):
				if index == 0:
					continue

				if index + 1 < len(words):
					pts.append([int(words[index]), int(words[index + 1])])

			# DEBUG:
			# print(pts)

			pts = np.array(pts)
			cv2.fillPoly(shapes, [pts], color=(255, 0, 0))

	# Generate output by blending image with shapes image, using the shapes
	# images also as mask to limit the blending to those parts
	out_rgb = img_rgb.copy()
	alpha = 0.5
	mask = shapes.astype(bool)
	out_rgb[mask] = cv2.addWeighted(img_rgb, alpha, shapes, 1 - alpha, 0)[mask]

	return out_rgb

	# cv2.imshow(f"{os.path.basename(img_rgb_path)}", img_rgb)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()


def main():
	# This is needed to avoid strange crashes related to opencv
	cv2.setNumThreads(0)

	folder_img_rbg   = "tss_out/tss_out_rgb_img"
	folder_out_ins   = "tss_out/tss_out_ins"
	folder_img_drawn = "tss_out/tss_out_drawn"

	img_rgb_list   = glob.glob(os.path.join(folder_img_rbg, "*.jpg"))

	for img_rgb_path in tqdm(img_rgb_list):
		basename       = os.path.basename(img_rgb_path)
		basename_noext = os.path.splitext(basename)[0]
		txt_ins_path   = os.path.join(folder_out_ins, f"{basename_noext}.txt")

		if os.path.exists(txt_ins_path):
			img_rgb = draw_instance(img_rgb_path, txt_ins_path)
			cv2.imwrite(os.path.join(folder_img_drawn, f"{basename_noext}.jpg"), img_rgb)

		# DEBUG:
		# break


if __name__ == '__main__':
	main()
