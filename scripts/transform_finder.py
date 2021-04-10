#!/usr/bin/env python3
import yaml
import logging
from math import sqrt
from pathlib import Path
from collections import namedtuple
import cv2 as cv
import numpy as np


Point = namedtuple("Point", ["x", "y"])
# 定位点，key 为序号
# 中心对称：1和10, 2和9
point_data = {
	0: Point(0, 0),          # 原点
	1: Point(4890, -11690),  # R4
	2: Point(4890, -2400),   # R3
	# 4: Point(13950, -14580), # B1
	9: Point(23110, -12960), # B3
	10: Point(23110, -3330), # B4
}

mouse_pos = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def mouse_callback(event, x, y, flags, param):
	if event == cv.EVENT_LBUTTONDBLCLK:
		global mouse_pos
		mouse_pos = (x, y)
		logger.debug("Double clicked: %d, %d", x, y)


def draw_axis(src, point):
	"""画 x 坐标轴方向
	"""
	length = 40
	thickness = 2
	color = (0, 0, 0)
	x, y = point.x, point.y
	ret = cv.line(src, (x, y), (x + length, y), color, thickness)
	ret = cv.line(ret, (x + length, y), (x + length - 10, y + 10), color, thickness)
	ret = cv.line(ret, (x + length, y), (x + length - 10, y - 10), color, thickness)
	return ret


def draw_border(src, thickness):
	"""画边框
	"""
	src[0:thickness, ::] = 0
	src[-thickness:, ::] = 0
	src[::, 0:thickness] = 0
	src[::, -thickness:] = 0
	return src


def main():
	windows_title = "Image layyer"
	# 场地俯视图路径
	image_path = "/home/hb/srm/SlamRobot/data/battleground-topview.png"
	# SLAM 生成的地图
	map_config_path = "/home/hb/srm/SlamRobot/data/map0407.yaml"
	map_path = None
	map_config = None
	with open(map_config_path, "r") as fp:
		map_config = yaml.load(fp)
		map_path = str(Path(map_config_path).parent / Path(map_config["image"]))
		logger.info("map_path: %s", map_path)

	origin_image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
	cv.imshow(windows_title, origin_image)
	cv.setMouseCallback(windows_title, mouse_callback)

	# 寻找场地原点在 origin_image 中的位置，一个像素的实际长度
	global mouse_pos
	pixel_point = dict()
	for i in point_data.keys():
		print("请双击 %d 号定位点" % i)
		mouse_pos = None
		while mouse_pos is None:
			k = cv.waitKey(100)
			if k == ord("q"):
				cv.destroyAllWindows()
				return
		pixel_point[i] = Point(*mouse_pos)
	# pixel_point = {
	# 	0: Point(x=112, y=50),
	# 	1: Point(x=373, y=642), 
	# 	2: Point(x=373, y=156), 
	# 	4: Point(x=831, y=787), 
	# 	9: Point(x=1294, y=707), 
	# 	10: Point(x=1294, y=220),
	# }
	logger.info("pixel_point: %s", pixel_point)
	# 1,2,9,10差分求分辨率
	resolution_x1 = (point_data[1].x - point_data[9].x) / (pixel_point[1].x - pixel_point[9].x)
	resolution_x2 = (point_data[2].x - point_data[10].x) / (pixel_point[2].x - pixel_point[10].x)
	resolution_y1 = (point_data[1].y - point_data[2].y) / (pixel_point[1].y - pixel_point[2].y)
	resolution_y2 = (point_data[9].y - point_data[10].y) / (pixel_point[9].y - pixel_point[10].y)
	resolution = (abs(resolution_x1) + abs(resolution_x2) + abs(resolution_y1) + abs(resolution_y2)) / 4
	# 求中心对陈点
	centeral_x = sum([pixel_point[i].x for i in [1, 10, 2, 9]]) / 4
	centeral_y = sum([pixel_point[i].y for i in [1, 10, 2, 9]]) / 4
	logger.info("resolution: %.2f", resolution)
	logger.info("centeral: Point(%d, %d)", centeral_x, centeral_y)

	origin_image = draw_axis(origin_image, pixel_point[0])

	print("请调节上层地图，使其和俯视图重叠。")
	print("a/z 缩放，s/x 旋转，hjkl 平移，w 确定，q 退出")
	map_image = cv.imread(map_path, cv.IMREAD_GRAYSCALE)
	# add border and axis
	map_image = draw_border(map_image, 2)
	map_frame_px = int(-map_config["origin"][0] / map_config["resolution"])
	map_frame_py = map_image.shape[0] - int(-map_config["origin"][1] / map_config["resolution"])
	map_image = draw_axis(map_image, Point(map_frame_px, map_frame_py))
	logger.debug("map_frame_px: %d, map_frame_py: %d", map_frame_px, map_frame_py)

	map_rows, map_cols = map_image.shape
	if map_rows > origin_image.shape[0] or \
		map_cols > origin_image.shape[1]:
		# TODO
		logger.critical("map_image width or height was excessive")
		cv.destroyAllWindows()
		return

	alpha = 0.5
	scale = 1.0
	yaw = 0
	offset_x = 0
	offset_y = 0
	rows, cols = map_image.shape

	tmp_image = 0xff + np.zeros(origin_image.shape, dtype=np.uint8)
	transformed_map = map_image.copy()
	start_row, end_row = 0, map_rows
	start_col, end_col = 0, map_cols
	while True:
		tmp_image.fill(0xff)
		cols = int(scale * map_cols)
		rows = int(scale * map_rows)
		transformed_map = cv.resize(map_image, (cols, rows))
		# 旋转变换
		M = cv.getRotationMatrix2D((cols // 2, rows // 2), yaw, 1.0)
		# transformed_map = cv.warpAffine(map_image, M2, (cols, rows))
		transformed_map = cv.warpAffine(transformed_map, M, (cols, rows))
		# 缩放
		logger.debug("offset_x: %d, offset_y: %d", offset_x, offset_y)
		# 底层矩形左上
		start_row = offset_y if offset_y >=0 else 0
		end_row = offset_y + rows
		end_row = end_row if end_row <= tmp_image.shape[0] else tmp_image.shape[0]
		start_col = offset_x if offset_x >= 0 else 0
		end_col = offset_x + cols
		end_col = end_col if end_col <= tmp_image.shape[1] else tmp_image.shape[1]
		# 相交区域左上
		rect_start_col, rect_start_row = max(0, -offset_x), max(0, -offset_y)
		# 相交区域右下
		rect_end_col = min(transformed_map.shape[1], tmp_image.shape[1] - offset_x)
		rect_end_row = min(transformed_map.shape[0], tmp_image.shape[0] - offset_y)
		tmp_image[start_row:end_row, start_col:end_col] = transformed_map[rect_start_row:rect_end_row, rect_start_col:rect_end_col]
		blend_image = cv.addWeighted(origin_image, alpha, tmp_image, (1 - alpha), 0)
		cv.imshow(windows_title, blend_image)

		k = cv.waitKey(0)
		logger.debug("Key %d pressed", k)
		if k == ord("q"):
			break
		# 平移（vim 键位）
		if k == ord("k"):
			offset_y -= 3
		elif k == ord("j"):
			offset_y += 3
		elif k == ord("h"):
			offset_x -= 3
		elif k == ord("l"):
			offset_x += 3
		# 缩放
		elif k == ord("a"):
			scale *= 1.01
		elif k == ord("z"):
			scale *= 0.99
		# 旋转
		elif k == ord("s"):
			yaw += 1
		elif k == ord("x"):
			yaw -= 1
		# 输出
		elif k == ord("w"):
			dx = (-pixel_point[0].x + offset_x) * resolution / 1000
			dy = (pixel_point[0].y - offset_y) * resolution / 1000
			dx = dx - map_config["origin"][0]
			dy = dy - (map_image.shape[0] * map_config["resolution"] + map_config["origin"][1])
			print("world -> map 转换")
			print("transform(x, y, yaw): %.2f %.2f %.2f" % (dx, dy, yaw / 180 * 3.14159))

	cv.destroyAllWindows()


if __name__ == "__main__":
	main()
