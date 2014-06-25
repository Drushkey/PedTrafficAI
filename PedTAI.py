import cv2
import cvutils
import numpy as np
import sqlite3
import math
import csv
import os.path
import subprocess
import random

#Settings
x = 153
include_homo_altitude_mod = 1
grouping_mod = 0 #Modify only grouping parameters
use_previous_point_correspondence = 1
weight_mota = 1 
weight_motp = 1 - weight_mota #Do not change!
max_iterations = 3000

#Project-specific parameters
metersperpixel = '0.016533'
worldfile = 'Floorplan_poly.png'
videoframefile = 'calib_snapshot.png'
point_corr_filename = 'ext-point-correspondence.txt'
storage_filename = 'storage_attempt4.csv'
video_filename = 'calib_0.mp4'
sqlite_filename = 'PedTAI_testrun.sqlite'
homo_filename = 'homography.txt'
mask_filename = 'calib_mask_2.png'
config_filename = 'PedTAI_testrun.cfg'
ground_truth_sqlite = 'ground_truth_2.sqlite'

#Optimization parameters
t_init = 20
max_match_dist = 1 #maximum distance for matching in meters
lamda = 0.2
emax = -100 #Threshold solution to consider optimization complete

#this is probably garbage
def simanneal(MOTA,MOTP,tracker_name):
	import os.path

	if os.path.isfile(storage_filename):
		with open('storage.csv', 'wb') as csvfile:
			csvfiller = csv.writer(csvfile, delimiter=' ')
			emptyrow = [-1000] * 19 + [500,0]
			csvfiller.writerow(emptyrow)
	#Generates new random config values
	storage = open(storage_filename,'w')

#Truncates values before they are written to the config file
def trunc(f,n):
	slen = len('%.*f' % (n,f))
	return str(f)[:slen]

def signer():
	#Randomly selects between 1 and -1
	signer = random.uniform(0,1)
	if signer < 0.5:
		return -1
	else:
		return 1

def changeprinter(param,first,second):
	#Prints changes when they occur
	#thing to print = TTP
	if first == second:
		ttp = param + ' unchanged'
	else:
		ttp = param
		ttp += ' changed from ' + str(first) + ' to ' + str(second)
		print ttp

#Finds neighbor solutions; modifies a number of parameters proportional to the current temperature
def neighbor_solution(temp,t_init,boolelev,prevsol,prevelev, gmod):
	print 'Generating random neighbor solution...'
	potential_changes = 19
	if boolelev == 1:
		potential_changes = 23

	n_changes = random.randint(1,3)
	u = 0
	values_to_change = []
	print 'Number of changes to config file: ' + str(n_changes)
	if gmod == 0:
		while u < n_changes:
			success = 0
			while success == 0:
				add = random.randint(0,potential_changes)
				if add in values_to_change:
					pass
				else:
					values_to_change.append(add)
					success = 1
					u += 1
	else:
		while u < n_changes:
			success = 0
			while success == 0:
				add = random.randint(14,potential_changes)
				if add in values_to_change or add == 3 or add == 16 or add == 17:
					pass
				#control for parameters to optimize later
				elif add == 4 or add == 9 or add == 11 or add == 12:
					pass
				else:
					values_to_change.append(add)
					success = 1
					u += 1

	rsolution = []
	if 0 in values_to_change:
		signa = random.uniform(0,1)
		if signa < 0.25:
			sign = 0.3
		elif signa >= 0.25 and signa < 0.5:
			sign = 0.5
		elif signa >= 0.5 and signa < 0.75:
			sign = 2
		else:
			sign = 3
		new0 = float(prevsol[0])*sign
		if new0 > 1:
			new0 = 1
		rsolution.append(trunc(new0,6)) #feature-quality
		#Divides or multiples old value by 2
		changeprinter('feature-quality',prevsol[0],new0)
	else:
		rsolution.append(prevsol[0])
	if 1 in values_to_change:
		new1 = float(prevsol[1]) + signer()*random.uniform(0,0.4)
		if new1 < 0:
			new1 = 0
		elif new1 > 10:
			new1 = 10
		rsolution.append(trunc(new1,6)) #min-feature-distanceklt, assumes max distance of 1 meter between features, probably excessive
		changeprinter('min-feature-distanceklt',prevsol[1],new1)
	else:
		rsolution.append(prevsol[1])
	if 2 in values_to_change:
		new2 = int(prevsol[2]) + signer()
		if new2 < 3:
			new2 = 4
		elif new2 > 10:
			new2 = 9
		rsolution.append(new2) #window-size
		changeprinter('window-size',prevsol[2],new2)
	else:
		rsolution.append(int(prevsol[2]))
	if 3 in values_to_change: #Currently impossible
		rsolution.append(trunc(random.uniform(0,1),6)) #k-param
	else:
		rsolution.append(prevsol[3])
	if 4 in values_to_change:
		new4 = int(prevsol[4]) + signer()
		if new4 < 1:
			new4 = 2
		elif new4 > 5:
			new4 = 4
		rsolution.append(new4) #pyramid-level
		print 'pyramid-level changed from ' + str(prevsol[4]) + ' to ' + str(new4)
	else:
		rsolution.append(int(prevsol[4]))
	if 5 in values_to_change:
		new5 = int(prevsol[5]) + signer()
		if new5 < 2:
			new5 = 3
		elif new5 > 4:
			new5 = 3
		rsolution.append(new5) #ndisplacement
		changeprinter('ndisplacement',prevsol[5],new5)
	else:
		rsolution.append(int(prevsol[5]))
	if 6 in values_to_change:
		new6 = float(prevsol[6]) + signer()*random.uniform(0,0.04)
		if new6 < 0:
			new6 = 0
		elif new6 > 1:
			new6 = 1
		rsolution.append(trunc(new6,6)) #min-feature-displacement
		changeprinter('min-feature-displacement',prevsol[6],new6)
	else:
		rsolution.append(prevsol[6])
	if 7 in values_to_change:
		new7 = float(prevsol[7]) + signer()*random.uniform(0,0.4)
		if new7 < 1:
			new7 = 1
		elif new7 > 3:
			new7 = 3
		rsolution.append(trunc(new7,6)) #acceleration-bound
		changeprinter('acceleration-bound',prevsol[7],new7)
	else:
		rsolution.append(prevsol[7])
	if 8 in values_to_change:
		new8 = float(prevsol[8]) + signer()*random.uniform(0,0.2)
		if new8 < 0:
			new8 = 0
		elif new8 > 1:
			new8 = 1
		rsolution.append(trunc(new8,6)) #deviation-bound
		changeprinter('deviation-bound',prevsol[8],new8)
	else:
		rsolution.append(prevsol[8])
	if 9 in values_to_change:
		new9 = int(prevsol[9]) + signer()
		if new9 < 0:
			new9 = 1
		elif new9 > 11:
			new9 = 10
		rsolution.append(new9) #smoothing-halfwidth
		changeprinter('smoothing-halfwidth',prevsol[9],new9)
	else:
		rsolution.append(prevsol[9])
	if 10 in values_to_change:
		new10 = int(prevsol[10]) + signer()
		if new10 < 1:
			new10 = 2
		rsolution.append(new10) #n-frames-velocity, not used for feature tracking
		changeprinter('n-frames-velocity',prevsol[10],new10)
	else:
		rsolution.append(int(prevsol[10]))
	if 11 in values_to_change:
		new11 = float(prevsol[11]) + signer()*random.uniform(0,0.04)
		if new11 < 0.01:
			new11 = 0.01
		elif new11 > 0.3:
			new11 = 0.3
		rsolution.append(trunc(new11,6)) #min-tracking-error
		changeprinter('min-tracking-error',prevsol[11],new11)
	else:
		rsolution.append(prevsol[11])
	if 12 in values_to_change:
		rsolution.append(0.0001) #min-eig-value, MAY NEED CHANGE
	else:
		rsolution.append(prevsol[12])
	if 13 in values_to_change:
		new13 = int(prevsol[13]) + signer()
		if new13 < 5:
			new13 = 6
		elif new13 > 25:
			new13 = 24
		rsolution.append(new13) #min-feature-time
		changeprinter('min-feature-time',prevsol[13],new13)
	else:
		rsolution.append(int(prevsol[13]))
	if 14 in values_to_change:
		new14 = float(prevsol[14]) + signer()*random.uniform(0,0.8)
		if new14 < 0.5:
			new14 = 0.5
		elif new14 > 4:
			new14 = 4
		rsolution.append(trunc(new14,6)) #mm-connection-distance
		changeprinter('mm-connection-distance',prevsol[14],new14)
	else:
		rsolution.append(prevsol[14])
	if 15 in values_to_change:
		mmsd = 5000
		while mmsd >= float(rsolution[-1]):
			mmsd = float(prevsol[15]) + signer()*random.uniform(0,0.4)
		if mmsd < 0.1:
			mmsd = 0.1
		rsolution.append(trunc(mmsd,6))
		changeprinter('mm-segmentation-distance',prevsol[15],mmsd)
	else:
		rsolution.append(prevsol[15])
	if 16 in values_to_change:
		new16 = float(prevsol[16]) + signer()*random.uniform(0,0.4)
		if new16 < 0:
			new16 = 0
		elif new16 > 5:
			new16 = 5
		rsolution.append(trunc(new16,6)) #max-distance, apparently unused
		changeprinter('max-distance',prevsol[16],new16)
	else:
		rsolution.append(prevsol[16])
	if 17 in values_to_change:
		#Unused
		rsolution.append(trunc(random.uniform(0,1),6)) #min-velocity-cosine, apparently unused
	else:
		rsolution.append(prevsol[17])
	if 18 in values_to_change:
		new18 = float(prevsol[18]) + signer()*random.uniform(0,0.4)
		if new18 < 1:
			new18 = 1
		elif new18 > 4:
			new18 = 4
		rsolution.append(trunc(new18,6)) #min-features-group
		changeprinter('min-features-group',prevsol[18],new18)
	else:
		rsolution.append(prevsol[18])

	if boolelev == 1:
		elevout = []
		if 19 in values_to_change:
			el0 = float(prevelev[0]) + signer()*random.uniform(0,0.4)
			if el0 < 0.5:
				el0 = 0.5
			elif el0 > 1.5:
				el0 = 1.5
			elevout.append(el0)
			changeprinter('Elevation 1',prevelev[0],el0)
		else:
			elevout.append(prevelev[0])
		if 20 in values_to_change:
			el1 = float(prevelev[1]) + signer()*random.uniform(0,0.4)
			if el1 < 0.5:
				el1 = 0.5
			elif el1 > 1.5:
				el1 = 1.5
			elevout.append(el1)
			changeprinter('Elevation 2',prevelev[1],el1)
		else:
			elevout.append(prevelev[1])
		if 21 in values_to_change:
			el2 = float(prevelev[2]) + signer()*random.uniform(0,0.4)
			if el2 < 0.5:
				el2 = 0.5
			elif el2 > 1.5:
				el2 = 1.5
			elevout.append(el2)
			changeprinter('Elevation 3',prevelev[2],el2)
		else:
			elevout.append(prevelev[2])
		if 22 in values_to_change:
			el3 = float(prevelev[3]) + signer()*random.uniform(0,0.4)
			if el3 < 0.5:
				el3 = 0.5
			elif el3 > 1.5:
				el3 = 1.5
			elevout.append(el3)
			changeprinter('Elevation 4',prevelev[3],el3)
		else:
			elevout.append(prevelev[3])
	else:
		elevout = [1.2,1.2,1.2,1.2]

	return rsolution, elevout

#Matches traces to ground-truth and calculates MOTP+MOTA
def matchmaking(object_positions,gt_filename,T):
	#Matches Oi to Hi
	conn = sqlite3.connect(gt_filename)
	gt_positionsx = conn.execute('SELECT * FROM positions')

	max_frame = 0
	n_gt_objects = 0
	n_obj = 0

	#Add boolean IsConnected to gt_positions
	gt_positions = []
	for g in gt_positionsx:
		gt_positions.append(g)

	#Compute number of objects and frames in ground truth
	for g in gt_positions:
		if g[0] > n_gt_objects:
			n_gt_objects = g[0]
		if g[1] > max_frame:
			max_frame = g[1]

	for op in object_positions:
		if op[0] > n_obj:
			n_obj = op[0]

	#Sort into easier tables
	sorted_gt_positions = [0]*(n_gt_objects+1)
	sorted_obj_positions = [0]*(n_obj+1)
	for x in range(0,n_gt_objects+1):
		sorted_gt_positions[x] = [[0,0],[0,0]]
	for x in range(0,n_obj+1):
		sorted_obj_positions[x] = [[0,0],[0,0]]
	for g in gt_positions:
		sorted_gt_positions[g[0]].append(g)
	for s in object_positions:
		sorted_obj_positions[s[0]].append(s)
	#Dirty array fixing
	for sgt in sorted_gt_positions:
		del sgt[0]
		sgt[0] = [sgt[1][1],sgt[-1][1]]
	for sop in sorted_obj_positions:
		del sop[0]
		if len(sop) >= 2:
			sop[0] = [sop[1][1],sop[-1][1]]
		else:
			return 0,0,0,0,0,0

	match_table = []
	frame = 0

	#Make matches and calculate distance
	while frame <= max_frame:
		for sgt in sorted_gt_positions:
			if frame >= sgt[0][0] and frame <= sgt[0][1]:
				for gt in sgt:
					if gt[1] == frame:
						best_dist = [frame,sgt[1][0],-1,float('inf')]
						for sop in sorted_obj_positions:
							if frame >= sop[0][0] and frame <= sop[0][1]:
								for s in sop:
									if len(s) == 2 or len(gt) == 2:
										continue
									elif s[1] == frame:
										delta = math.sqrt(math.pow((s[2] - gt[2]),2) + math.pow((s[3] - gt[3]),2))
										if delta >= T:
											continue
										else:
											if delta < best_dist[3]:
												best_dist = [frame,gt[0],s[0],delta]
											break
						match_table.append(best_dist)
						break
		frame += 1

	#Calculate MOTP
	dit = 0
	ct = 0
	mt = 0
	for mtab in match_table:
		if mtab[2] != -1:
			dit += float(mtab[3])/T
			ct += 1
		else:
			mt += 1
	if ct != 0:
		motp = 1 - (dit/ct)
	else:
		return 0,0,0,0,0,0

	#Calculate MOTA
	gt = 0
	for sgt in sorted_gt_positions:
		gt += (len(sgt)-1)

	total_traces = len(object_positions)

	fpt = total_traces - ct

	gtobj = 0
	mme = 0
	while gtobj <= n_gt_objects:
		prev = [0,0,-1,0]
		new_match = 0
		for mtab in match_table:
			if mtab[1] == gtobj:
				if new_match == 0:
					new_match = 1
					mme = mme - 1
				if mtab[2] != prev[2]:
					mme += 1
				prev = mtab
		gtobj += 1

	mota = 1-(float(mt+fpt+mme)/gt)

	print 'MOTP: ' + str(motp)
	print 'MOTA: ' + str(mota)
	return motp, mota, dit, mt, mme,fpt

#Extracts trajectories from the sqlite file	
def extract_trajectories(sqlite_filename):
	import storage

	objects = storage.loadTrajectoriesFromSqlite(sqlite_filename,'object')
	object_positions = []
	a = 0
	for o in objects:
		instant = o.timeInterval[0]
		for p in o.positions:
			object_positions.append([a,instant,p.x,p.y])
			instant += 1
		a += 1

	return object_positions

#Writes the cfg file for current parameters
def config_mod(param_array, vidfilename,datafilename,homofilename,maskfilename,tracker_filename):
	cfg = open(tracker_filename, 'w')
	cfg.write('# Automatically generated configuration file for Traffic Intelligence\n')
	cfg.write('video-filename = ')
	cfg.write(vidfilename)
	cfg.write('\ndatabase-filename = ')
	cfg.write(datafilename)
	cfg.write('\nhomography-filename = ')
	cfg.write(homofilename)
	cfg.write('\nmask-filename = ')
	cfg.write(maskfilename)
	cfg.write('\nload-features = false\ndisplay = false\nvideo-fps = 29.97\nmeasurement-precision = 3')
	cfg.write('\nframe1 = 0\nnframes = 0\nmax-nfeatures = 1000\nfeature-quality = ')
	cfg.write(str(param_array[0]))
	cfg.write('\nmin-feature-distanceklt = ')
	cfg.write(str(param_array[1]))
	cfg.write('\nwindow-size = ')
	cfg.write(str(param_array[2]))
	cfg.write('\nuse-harris-detector = false\nk = ')
	cfg.write(str(param_array[3]))
	cfg.write('\npyramid-level = ')
	cfg.write(str(param_array[4]))
	cfg.write('\nndisplacements = ')
	cfg.write(str(param_array[5]))
	cfg.write('\nmin-feature-displacement = ')
	cfg.write(str(param_array[6]))
	cfg.write('\nacceleration-bound = ')
	cfg.write(str(param_array[7]))
	cfg.write('\ndeviation-bound = ')
	cfg.write(str(param_array[8]))
	cfg.write('\nsmoothing-halfwidth = ')
	cfg.write(str(param_array[9]))
	cfg.write('\nnframes-velocity = ')
	cfg.write(str(param_array[10]))
	cfg.write('\nmax-number-iterations = 20\nmin-tracking-error = ')
	cfg.write(str(param_array[11]))
	cfg.write('\nmin-feature-eig-threshold = ')
	cfg.write(str(param_array[12]))
	cfg.write('\nmin-feature-time = ')
	cfg.write(str(param_array[13]))
	cfg.write('\nmm-connection-distance = ')
	cfg.write(str(param_array[14]))
	cfg.write('\nmm-segmentation-distance = ')
	cfg.write(str(param_array[15]))
	cfg.write('\nmax-distance = ')
	cfg.write(str(param_array[16]))
	cfg.write('\nmin-velocity-cosine = ')
	cfg.write(str(param_array[17]))
	cfg.write('\nmin-nfeatures-group = ')
	cfg.write(str(param_array[18]))
	cfg.write('\nmax-predicted-speed = 50\nprediction-time-horizon = 5\ncollision-distance = 1.8\n')
	cfg.write('crossing-zones = false\nprediction-method = na\nnpredicted-trajectories = 10\nmin-acceleration = -9.1\n')
	cfg.write('max-acceleration = 2\nmax-steering = 0.5\nuse-features-prediction = true\nmax-normal-acceleration = 2\nmax-normal-steering = 2\nmin-extreme-acceleration = 2\nmax-extreme-acceleration = 3\nmax-extreme-steering = 3')

#Calculates homography for the current elevation
def point_corresp_mod(pointcorr_name,current_elevation,homo_filename):
	print 'Calculating homography matrix for current elevation...'
	elevprop = []
	for ce in current_elevation:
		elevprop.append(float(ce)/1.5)

	pct = open(pointcorr_name,'r')
	fullextract = pct.readlines()
	pclines = fullextract[-6::]
	video_lines = []

	worldPts = []
	temp_holder = []

	#Extract latest point correspondences
	for j in range(0,2):
		temp_holder.append(pclines[j].split())

	#Extract world points
	for k in range(0,4):
		worldPts.append([float(temp_holder[0][k]),float(temp_holder[1][k])])

	worldPts2 = np.float32(worldPts)
	
	#Prepare video point arrays
	for x in range(2,6):
		video_lines.append(pclines[x].split())
		for y in range(0,4):
			video_lines[x-2][y] = video_lines[x-2][y].split('e+')

	point_arrays = []
	#each point:
	# [[X0, Y0]
	# [X1, Y1]]
	for a in range (0,4):
		point_arrays.append([[float(video_lines[0][a][0])*(10**float(video_lines[0][a][1])),
			float(video_lines[1][a][0])*(10**float(video_lines[1][a][1]))],
			[float(video_lines[2][a][0])*(10**float(video_lines[2][a][1])),
			float(video_lines[3][a][0])*(10**float(video_lines[3][a][1]))]])

	curr_videoPts = []
	for i in range (0,4):
		delta_x = point_arrays[i][1][0] - point_arrays[i][0][0]
		delta_y = point_arrays[i][1][1] - point_arrays[i][0][1]
		curr_videoPts.append([point_arrays[i][0][0] + (delta_x * elevprop[i]),point_arrays[i][0][1] + (delta_y * elevprop[i])])

	curr_videoPts2 = np.float32(curr_videoPts)
	
	homography, mask = cv2.findHomography(np.array(curr_videoPts2), np.array(worldPts2))
	np.savetxt(homo_filename,homography)

def run_TI(configfile):
	print 'Running TrafficIntelligence'
	tfrun = 'feature-based-tracking ' + configfile + ' --tf'
	gfrun = 'feature-based-tracking ' + configfile + ' --gf'
	subprocess.check_call(tfrun, shell=True)
	print 'Feature traces extracted.'
	subprocess.check_call(gfrun, shell=True)
	print 'Features grouped.'

#Initialize csv file if none exists
if os.path.isfile(storage_filename):
	print 'Previous results found - continuing.'
	if include_homo_altitude_mod == 1:
		prevelev = []
		if use_previous_point_correspondence == 0:
			pc_run = 'python inputPointCorrespondence.py -i ' + videoframefile + ' -w ' + worldfile + ' -u ' + str(metersperpixel)
			subprocess.check_call(pc_run, shell=True)
			prevelev = [1.2,1.2,1.2,1.2]
		else:
			elevreader2 = []
			with open(storage_filename, 'rb') as storagefile:
				elevreader = csv.reader(storagefile, delimiter=' ')
				for row in elevreader:
					elevreader2.append(row)
				for x in range(20,24):
					prevelev.append(float(elevreader2[-1][x]))
				
		point_corresp_mod(point_corr_filename,prevelev,homo_filename)
else:
	print 'Initializing with default parameters.'
	current_config = [0.5,5,5,0.5,3,3,0.05,2,0.5,6,5,0.15,0.0001,15,2,1,2.5,0.5,2.5]
	config_mod(current_config,video_filename,sqlite_filename,homo_filename,mask_filename,config_filename)
	if os.path.isfile(sqlite_filename):
		removal = 'rm ' + sqlite_filename
		subprocess.check_call(removal, shell=True)
	run_TI(config_filename)
	current_traces = extract_trajectories(sqlite_filename)
	motp,mota,dit,mt,mme,fpt = matchmaking(current_traces,ground_truth_sqlite,max_match_dist)

	with open(storage_filename, 'wb') as storagefile:
		csvfiller = csv.writer(storagefile, delimiter=' ')
		uid = []
		for cc in current_config:
			uid.append(str(cc))
		prevelev = [1.2,1.2,1.2,1.2]
		firstrow = [0] + current_config + prevelev + [motp,mota,uid,0]
		csvfiller.writerow(firstrow)
	print 'Initial solution found. Beginning simulated annealing...'

#Initialize homography, if elevation is variable (otherwise should be done manually)
prevelev = []
if include_homo_altitude_mod == 1:
	prevelev = []
	if use_previous_point_correspondence == 0:
		pc_run = 'python inputPointCorrespondence.py -i ' + videoframefile + ' -w ' + worldfile + ' -u ' + str(metersperpixel)
		subprocess.check_call(pc_run, shell=True)
		prevelev = [1.2,1.2,1.2,1.2]
	else:
		elevreader2 = []
		with open(storage_filename, 'rb') as storagefile:
			elevreader = csv.reader(storagefile, delimiter=' ')
			for row in elevreader:
				elevreader2.append(row)
			prevelev = elevreader2[-1][20:24]
	point_corresp_mod(point_corr_filename,prevelev,homo_filename)
	
else:
	currelev = [1.2,1.2,1.2,1.2]

	#Initial solution

#In-app array of csv solutions
solutions = []
with open(storage_filename, 'rb') as storagefile:
	csvreader = csv.reader(storagefile, delimiter=' ')
	for row in csvreader:
		solutions.append(row)

#Extract last solution
prevsol = solutions[(int(solutions[-1][-1]))][1:20]
prevelev = solutions[(int(solutions[-1][-1]))][20:24]

i = int(solutions[-1][0])+1
e = weight_mota*(float(solutions[-1][-3])) + weight_motp*float(solutions[-1][-4])
print 'Current energy:'
print e
ebest = weight_mota*float(solutions[int(solutions[-1][-1])][-3]) + weight_motp*float(solutions[int(solutions[-1][-1])][-4])
currbest = solutions[-1][-1]

while i < max_iterations:
	print 'iteration : ' + str(i)
	if os.path.isfile(sqlite_filename):
		removal = 'rm ' + sqlite_filename
		subprocess.check_call(removal, shell=True)
	t = t_init - (lamda * math.log(1+i))
	print 'Temperature : ' + str(t)
	currsol, currelev = neighbor_solution(t,t_init,include_homo_altitude_mod,prevsol,prevelev,grouping_mod)
	print 'Current parameters :' + str(currsol)
	print 'Current elevations :' + str(currelev)
	uid = ''
	for cs in currsol:
		uid += str(cs)
	point_corresp_mod(point_corr_filename,currelev,homo_filename)
	config_mod(currsol,video_filename,sqlite_filename,homo_filename,mask_filename,config_filename)
	run_TI(config_filename)
	current_traces = extract_trajectories(sqlite_filename)
	motp,mota,dit,mt,mme,fpt = matchmaking(current_traces,ground_truth_sqlite,max_match_dist)
	
	enew = weight_mota*float(mota) + weight_motp*motp
	print 'New energy : '
	print enew
	if enew > ebest and enew != 0:
		print 'NEW BEST!!!'
		currbest = i
		ebest = enew
		saved_best_name = sqlite_filename[0:6]+'best.sqlite'
		if os.path.isfile(saved_best_name):
			remove_command = 'rm ' + saved_best_name
			subprocess.check_call(remove_command, shell=True)
		move_command = 'mv ' + sqlite_filename + ' ' + saved_best_name
		subprocess.check_call(move_command, shell=True)
	initprob = math.exp((t*x*enew)) / math.exp((t*x*e))
	if initprob > 1:
		initprob = 1
	print 'Probability to move : ' + str(initprob)
	probcompare = random.uniform(0,1)
	if initprob > probcompare and enew != 0:
		print 'Moved.'
		prevsol = currsol
		e = enew
		
	solutions.append([i]+currsol+currelev+[dit,mt,mme,fpt]+[motp,mota,uid,currbest])
	if os.path.isfile(storage_filename):
		removal = 'rm ' + storage_filename
		subprocess.check_call(removal, shell=True)
		with open(storage_filename, 'wb') as storagefile:
			csvfiller = csv.writer(storagefile, delimiter=' ')
			for sol in solutions:
				csvfiller.writerow(sol)
	
	i+=1


#test_insert = extract_trajectories('calib.sqlite')
#print matchmaking(test_insert,'Ground Truth v1.sqlite',1.5)

#Array contents
#	0 - feature-quality
#	1 - min-feature-distanceklt
#	2 - window-size
#	3 - k parameter
#	4 - pyramid-level
#	5 - ndisplacement
#	6 - min-feature-displacement
#	7 - acceleration-bound
#	8 - deviation-bound
#	9 - smoothing-halfwidth
#	10 - nframes-velocity
#	11 - min-tracking-error
#	12 - min-feature-eig-threshold
#	13 - min-feature-time
#	14 - mm-connection-distance
#	15 - mm-segmentation-distance
#	16 - max-distance
#	17 - min-velocity-cosine
#	18 - min-features-group

#Variables for testing
#curr_parameters = [0.01,3,6,0.2,5,3,0.05,3,0.6,5,5,0.3,0.0001,20,3.75,1.5,5,0.8,3]
#video_filename = 'GP010010.MP4'
#database_filename = 'Test1.sqlite'
#homography_filename = 'TestCaseHomo.txt'
#mask_filename = 'mask1.png'
#current_elevation = [0.5,0.5,0.5,0.5]

#ext_point_corr_filename = 'ext-point-correspondence.txt'

#config_mod(curr_parameters,video_filename,database_filename,homography_filename,mask_filename)
#point_corresp_mod(ext_point_corr_filename, current_elevation)

#NOTES
#Changed lamda from 1 to 0.01 (temperature reached 0 before 200 iterations)
#Changed lamda from 0.01 to 0.1 (temperature change too slow)
#Changed neighbor_solution to increment slowly. Capped it at 3 parameters changed per iteration.