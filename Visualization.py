

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import math
import numpy as np
import open3d as o3d

s = serial.Serial(port = 'COM6', baudrate = 115200, timeout = 10)
f = open("coordinates.xyz", "w") 
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission

s.write(b's')
rad = 0
z_coord = 0

# recieve 96 measurements from UART of MCU
for i in range(96):
    x = s.readline()
    distance = int(x.decode())
    print(distance, i)
    f.write(str( str(z_coord)+ " " + str(round(distance * math.cos(math.radians(float(rad))), 4)) + " " + str(round(distance * math.sin(math.radians(float(rad))), 4)) + "\n"))
    rad += 11.25

    if (i % 32 == 0):
        if (i != 0):
        z_coord = z_coord + 1000

    if (rad == 360):
        rad = 0

f.close()
s.close()


       
pcd = o3d.io.read_point_cloud("coordinates.xyz", format='xyz')

lines = []

# create loops
for z_coord in range(5):

    offset = 32 * z

    i = 0

    while i < 32:
        lines.append([i + offset, i+1 + offset])
        i += 1


    lines.append([i + offset, offset])



# connect the loops
for z_coord in range(5-1):
    startIndex = z_coord*32
    nextStartIndex = (z_coord+1)*32

    i = 0
    while i < 32:
        lines.append([startIndex+i, nextStartIndex+i])
        i += 1


pts = np.asarray(pcd.points)
print(pts)


line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(pts),
    lines=o3d.utility.Vector2iVector(lines),
)

o3d.visualization.draw_geometries([line_set], width=1280, height=720)



