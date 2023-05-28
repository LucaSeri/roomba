#!/usr/bin/python3

import socket
import matplotlib.pyplot as plt
import numpy as np
import time
import keyboard
import select
import pygame  # Used to read the joystick
import math
import threading

def send_controls():
		UDP_IP = "0.0.0.0"  # Listen on all available interfaces
		UDP_PORT = 5001 

		# Create a UDP socket
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		# Enable broadcast option
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

		# Bind the socket to a specific port
		sock.bind((UDP_IP, UDP_PORT))

		while True:
				# Update the joystick
				pygame.event.get()

				# Read the y values of the two joysticks normalized to [-1, 1]
				left_speed = -joystick.get_axis(1) * 255
				right_speed = -joystick.get_axis(4) * 255

				left_speed = int(left_speed)
				right_speed = int(right_speed)

				data = f"{left_speed} {right_speed}"

				# Send data as a UDP broadcast packet
				broadcast_addr = "255.255.255.255"
				sock.sendto(data.encode(), (broadcast_addr, UDP_PORT))
				time.sleep(0.05)


def receive_packets():
		UDP_IP = "0.0.0.0"  # Listen on all available interfaces
		UDP_PORT = 5000 

		# Create a UDP socket
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		# Bind the socket to a specific port
		sock.bind((UDP_IP, UDP_PORT))

		fig, ax = plt.subplots()

		ax.set_xlim(-100, 100)
		ax.set_ylim(-100, 100)

		line, = ax.plot([], [], 'b-')  # Line plot for the path
		obstacles = ax.scatter([], [], color='red')  # Scatter plot for obstacles

		x_points = []
		y_points = []

		obstacle_x_points = []
		obstacle_y_points = []

		minx = -10
		maxx = 10
		miny = -10
		maxy = 10

		while True:
				ready, _, _ = select.select([sock], [], [], 0.1)

				data, _ = sock.recvfrom(1024)  # Buffer size is 1024 bytes
				x, y, theta, distance, sensor = data.decode().split()

				sensor = int(sensor)

				angle = 0 if sensor == 0 else math.pi / 2 if sensor == 1 else -math.pi / 2
				x = float(x)
				y = -float(y)
				theta = float(theta)
				distance = float(distance)

				print(
				    f"x: {x}, y: {y}, theta: {theta * 180 / math.pi}, distance: {distance}, sensor: {sensor}, angle: {angle * 180 / math.pi}",
				    end="\r",
				)

				x_points.append(float(x))
				y_points.append(float(y))

				maxx = max(maxx, x)
				minx = min(minx, x)
				maxy = max(maxy, y)
				miny = min(miny, y)

				if 0 < distance < 80:
						obstacle_x = x + distance * math.cos(theta + angle)
						obstacle_y = -y + distance * math.sin(theta + angle)

						obstacle_y = -obstacle_y

						obstacle_x_points.append(obstacle_x)
						obstacle_y_points.append(obstacle_y)

				ax.set_xlim(miny - 10, maxy + 10)
				ax.set_ylim(minx - 10, maxx + 10)

				# Rotate the map such that the robot is always facing up

				rotated_y_points = []
				rotated_x_points = []

				for x, y in zip(x_points, y_points):
						rotated_x = x * math.cos(theta) - y * math.sin(theta)
						rotated_y = x * math.sin(theta) + y * math.cos(theta)

						rotated_x_points.append(rotated_x)
						rotated_y_points.append(rotated_y)


				line.set_data(rotated_y_points, rotated_x_points)

				rotated_obstacle_x_points = []
				rotated_obstacle_y_points = []

				for x, y in zip(obstacle_x_points, obstacle_y_points):
						rotated_x = x * math.cos(theta) - y * math.sin(theta)
						rotated_y = x * math.sin(theta) + y * math.cos(theta)

						rotated_obstacle_x_points.append(rotated_x)
						rotated_obstacle_y_points.append(rotated_y)	

				minx = min(rotated_obstacle_x_points + rotated_x_points)
				maxx = max(rotated_obstacle_x_points + rotated_x_points)
				miny = min(rotated_obstacle_y_points + rotated_y_points)
				maxy = max(rotated_obstacle_y_points + rotated_y_points)

				ax.set_xlim(miny - 10, maxy + 10)
				ax.set_ylim(minx - 10, maxx + 10)	

				obstacles.set_offsets(np.c_[rotated_obstacle_y_points, rotated_obstacle_x_points])


				plt.draw()
				plt.pause(0.0001)

def receive_packets_old():
		UDP_IP = "0.0.0.0"  # Listen on all available interfaces
		UDP_PORT = 5000 

		# Create a UDP socket
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		# Bind the socket to a specific port
		sock.bind((UDP_IP, UDP_PORT))

		fig, ax = plt.subplots()

		ax.set_xlim(-100, 100)
		ax.set_ylim(-100, 100)

		scatter = ax.scatter([], [])
		plt.draw()

		x_points = []
		y_points = []

		obstacle_x_points = []
		obstacle_y_points = []

		minx = -10
		maxx = 10
		miny = -10
		maxy = 10

		while True:
				# Check non-blocking for a UDP packet on port 5000 from any IP
				# This is non-blocking
				ready, _, _ = select.select([sock], [], [], 0.1)

				data, _ = sock.recvfrom(1024)  # Buffer size is 1024 bytes
				x, y, theta, distance, sensor = data.decode().split()

				sensor = int(sensor)

				angle = 0 if sensor == 0 else math.pi / 2 if sensor == 1 else -math.pi / 2

				x = float(x)
				y = -float(y)
				theta = float(theta)
				distance = float(distance)

				print(
						f"x: {x}, y: {y}, theta: {theta * 180 / math.pi}, distance: {distance}, sensor: {sensor}, angle: {angle * 180 / math.pi}",
						end="\r",
				)

				# Plot the x, y points
				x_points.append(float(x))
				y_points.append(float(y))

				maxx = max(maxx, x)
				minx = min(minx, x)
				maxy = max(maxy, y)
				miny = min(miny, y)

				if 0 < distance < 30:
						obstacle_x = x + distance * math.cos(theta + angle)
						obstacle_y = -y + distance * math.sin(theta + angle)

						obstacle_y = -obstacle_y

						obstacle_x_points.append(obstacle_x)
						obstacle_y_points.append(obstacle_y)

						maxx = max(maxx, obstacle_x)
						minx = min(minx, obstacle_x)
						maxy = max(maxy, obstacle_y)
						miny = min(miny, obstacle_y)

				ax.set_xlim(miny - 10, maxy + 10)
				ax.set_ylim(minx - 10, maxx + 10)

				# Plot the path in blue and the obstacle in red
				scatter.set_offsets(list(zip(y_points, x_points)))
				scatter.set_color("blue")

				_ = ax.scatter(obstacle_y_points, obstacle_x_points, color="red")


				# Change the plot size based on the points
				plt.draw()
				plt.pause(0.0001)

				time.sleep(0.1)


# Initialize the pygame library
pygame.init()

# Initialize the joystick
pygame.joystick.init()

# Check for joystick availability
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
		print("No joysticks available")
		exit()

# Use joystick #0 and initialize it
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Get the name from the OS for the controller/joystick
name = joystick.get_name()
print(f"Joystick name: {name}")

# Create and start the threads
send_thread = threading.Thread(target=send_controls)
receive_thread = threading.Thread(target=receive_packets)
send_thread.start()
receive_thread.start()
