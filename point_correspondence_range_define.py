import sqlite3
import cv2
import numpy as np

conn = sqlite3.connect('ground_truth_2.sqlite')

boxes = conn.execute('SELECT * FROM bounding_boxes')
averaged_table = []

for row in boxes:
	new_row = [row[0], row[1]]
	new_row.append((row[2] + row[4])/2)
	new_row.append((row[3] + row[5])/2)
	averaged_table.append(new_row)

pointsToTranslate = []
for line in averaged_table:
	pointsToTranslate.append([line[0],line[1],line[2],line[3]])

a = np.array(pointsToTranslate, dtype='float32')
a = np.array([a])

f = open('homography.txt','r')
homoMatrix = []
for txtline in f.readlines():
	temptxt = txtline[:-1].split()
	homoMatrix.append(temptxt)

i,j = 0,0
hh = [[0,0,0],[0,0,0],[0,0,0]]
for x in homoMatrix:
	for y in x:
		hh[i][j] = float(y)
		j += 1
		if j == 3:
			j = 0
			i += 1

h = np.array(hh, dtype='float32')

translatedPoints = []
print hh
for ptt in pointsToTranslate:
	w = hh[2][0]*ptt[2] + hh[2][1]*ptt[3] + hh[2][2]
	if w != 0:
		x = (hh[0][0]*ptt[2] + hh[0][1]*ptt[3] + hh[0][2])/w
		y = (hh[1][0]*ptt[2] + hh[1][1]*ptt[3] + hh[1][2])/w
	else:
		x = 0
		y = 0
	translatedPoints.append([ptt[0],ptt[1],x,y])

#translatedPoints = cv2.perspectiveTransform(a,h)

print translatedPoints

#conn.execute('''DROP TABLE positions''')
conn.execute('''CREATE TABLE positions
	(object_id INTEGER,
	frame_number INTEGER,
	x_coord REAL,
	y_coord REAL);''')

for r in translatedPoints:
	conn.execute('''INSERT INTO positions (object_id, frame_number, x_coord, y_coord) VALUES (?,?,?,?);''', (int(r[0]),int(r[1]),float(r[2]),float(r[3])))

conn.commit()
conn.close()