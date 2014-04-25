def config_mod(param_array, vidfilename,datafilename,homofilename,maskfilename):
	cfg = open('tracking.cfg', 'w')
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
	cfg.write('\nndisplacement = ')
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
	cfg.write('\nmin-features-group = ')
	cfg.write(str(param_array[18]))
	cfg.write('\nmax-predicted-speed = 50\nprediction-time-horizon = 5\ncollision-distance = 1.8\n')
	cfg.write('crossing-zones = false\nprediction-method = na\nnpredicted-trajectories = 10\nmin-acceleration = -9.1\n')
	cfg.write('max-acceleration = 2\nmax-steering = 0.5\nuse-features-prediction = true')

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
curr_parameters = [0.01,3,6,0.2,5,3,0.05,3,0.6,5,5,0.3,0.0001,20,3.75,1.5,5,0.8,3]
video_filename = 'GP010010.MP4'
database_filename = 'Test1.sqlite'
homography_filename = 'TestCaseHomo.txt'
mask_filename = 'mask1.png'

config_mod(curr_parameters,video_filename,database_filename,homography_filename,mask_filename)