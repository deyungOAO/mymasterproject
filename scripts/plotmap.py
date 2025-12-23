#!/usr/bin/env python3
from PIL import Image, ImageDraw

def meters_to_pixels(x_m, y_m, resolution, height_px):
	"""
	Convert map meters (0,0 at bottom-left; x right, y up)
	to image pixels (0,0 at top-left; x right, y down).
	"""
	x_px = x_m / resolution
	y_px = height_px - (y_m / resolution)
	return x_px, y_px

def main():
	# ============================
	# Map size in meters
	# ============================
	width_m  = 40.0		# road length in x
	height_m = 20.0		# road width in y

	# Resolution (meters per pixel)
	resolution = 0.05	# 5 cm / pixel

	# Image size in pixels
	width_px  = int(width_m / resolution)
	height_px = int(height_m / resolution)

	print(f"Image size: {width_px} x {height_px} pixels")

	# Create white background (free space / road)
	img = Image.new('L', (width_px, height_px), 255)
	draw = ImageDraw.Draw(img)

	# ============================
	# Obstacles (from your SDF),
	# defined in MAP METERS
	# ============================

	# Rectangles: (x_min_m, y_min_m, x_max_m, y_max_m)
	rectangles_m = [
		# building_block: full top band
		(0.0, 14.0, 40.0, 20.0),

		# big_lawn: full bottom band
		(0.0, 0.0, 40.0, 5.0),

		# small_lawn: small patch on upper-left area
		(8.5, 12.5, 13.5, 17.5),
	]

	# Circles: (cx_m, cy_m, radius_m)
	circles_m = [
		# road block: big cylinder
		(15.0, 14.0, 3.0),

		# lamp_pole_1: small post
		(24.0, 6.0, 0.05),
	]

	# ============================
	# Draw rectangles (black)
	# ============================
	for (x_min_m, y_min_m, x_max_m, y_max_m) in rectangles_m:
		x1_px, y1_px = meters_to_pixels(x_min_m, y_min_m, resolution, height_px)
		x2_px, y2_px = meters_to_pixels(x_max_m, y_max_m, resolution, height_px)

		# y is flipped already in meters_to_pixels, so we swap y1/y2
		draw.rectangle([x1_px, y2_px, x2_px, y1_px], fill=0)

	# ============================
	# Draw circles (black)
	# ============================
	for (cx_m, cy_m, r_m) in circles_m:
		cx_px, cy_px = meters_to_pixels(cx_m, cy_m, resolution, height_px)
		r_px = r_m / resolution

		bbox = [
			cx_px - r_px, cy_px - r_px,
			cx_px + r_px, cy_px + r_px
		]
		draw.ellipse(bbox, fill=0)

	# ============================
	# Save PNG
	# ============================
	output_file = "campus_road_40x20.png"
	img.save(output_file)
	print(f"Saved {output_file}")

if __name__ == "__main__":
	main()
