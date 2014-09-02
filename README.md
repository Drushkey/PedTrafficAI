#PEDESTRIAN TRAFFIC AI
- Dariush Ettehadieh, Polytechnique Montreal

The aim of this project is to calibrate video-based trackers via simulated annealing for pedestrian tracking in a given video.


## 1 - INSTRUCTION SUMMARY

1 - Prepare a short segment (~1 minute) of the video the tracker is to be applied to.

2 - Extract the ground-truth. A good tool for this step is JP Jodoin's annotation app : http://www.jpjodoin.com/urbantracker/tools.html

3 - If calculating homography, extract a single example frame from the video, as well as a floorplan/satellite image/other to-scale plan of the area.

4 - Prepare the following three configuration files (detailed instructions for this step are below):
	- setup.ini
	- staticParameters.txt
	- variableParameters.txt

5 - If using a tracker other than Traffic Intelligence, modify the extract_trajectories function in the code (TrOPed.py - see section 3 for details).

6 - Run TroPed.py with "py TroPed.py". Optimization may take up to several days.

7 - The resulting optimized parameters can now be used for tracking on the initial video. Optimal configuration files can be generated automatically with **TrOPedFin.py**, then requiring only the video filename be modified by the user. The tracker can then be run as normal.

## 2 - Setup file instructions

###setup.ini parameters

This file defines the parameters of the optimization process.

[ConfigFiles]

**nConfigs**: The number of configuration files required by the tracker [between 1 and 4]

**config0-config4**: Names or filepaths of the configuration files [string]

[HomographyOptions] *Settings for the treatment/use of homography. If no_homography is set to 1, the other settings can be ignored*

**no_homography**: Whether to not compute a homography matrix or apply homography to the ground truth [1/0]

**include_homo_altitude_mod**: Includes the elevation of points used for calculation of the homography matrix as an additional parameter. This runs inputPointCorrespondence.py, in which 4 points must be established at ground level and above (in the same order) in the image space, as well as in the world-space from the provided image. [1/0]

**shift_gt_homo**: Whether to include the elevation difference between tracker and ground-truth homographies as an additional parameter. This allows for compensation if the tracker and ground-truth are expected to detect pedestrians at different elevations (e.g. head vs center of mass). [1/0]

**metersperpixel**: Number of meters per pixel in the world-space. [float]

**homo_filename**: Filename/path for creation of the homography matrix file.

**point_corr_filename**: Filename/path for creation of the point correspondence file.

**gthomo_filename**: Filename/path for creation of the ground-truth homography. Must be defined even if shift_gt_homo is set to 0.

**videoframefile**: Filename/path of a frame from the video, used for point-correspondence.

**worldfile**: Filename/path of the world image - either a floorplan, satellite image, or other to-scale representation of the area in the video - for point correspondence.

[RunSettings] *Settings related to running the tracker from within the optimization*

**nrunlines**: Number of command-line commands required to run the tracker in its entirety [1-4]

**runline0-runline3**: Command-lines to run the tracker or parts thereof, in order [string]

[GeneralSettings]

**weight_mota**: Weight of Measure Of Tracking Accuracy in the calculation of energy for a given state; the weight of MOTP (Precision) is derived from this. It is suggested to keep this value near 1 at the initial stages of optimization [float, 0-1]

**max_iterations**: Maximum number of iterations to perform [int]

**relative_change**: Factor used to fine-tune neighbor-state generation at later phases of optimization, multiplied by the change-quantities set in varParams.txt [float]

**max_n_changes**: Maximum number of parameters to change per iteration.

**storage_filename**: Filename/path of the csv file in which optimization results will be kept.

**video_filename**: Filename/path of the video file.

**ground_truth_sqlite**: Filename/path of the ground-truth tracks.

**sqlite_filename**: Filename/path of the tracker output.

[OptimizationParameters]

**prob_constant**: Constant in the state-acceptance equation. Higher values make regressing to lower MOTA/MOTP less likely, and vice-versa. [float]

**t_init**: Starting temperature for simulated annealing [float]

**max_match_dist**: Maximum distance between tracker and ground-truth tracks for a match to be recorded. In meters if using homography, otherwise in pixels.

**lamda**: Constant for temperature change at every iteration; higher values increase how quickly the algorithm becomes less tolerant to lower MOTA/MOTP iterations.

**emax**: Minimum energy. Currently unused.

###variableParameters.txt

This file defines the tracker parameters to be optimized, in CSV format. Example:

*0,feature-quality,float,ratio,0.5,0.000001,1,2*
*0,min-feature-distanceklt,float,add,5,0,10,0.4*

The parameters are as follows, in order:
	- configuration file in which to write the parameter, according to the numbering defined in setup.ini [int, 0-3]
	- name of the parameter, as written in the configuration file [string]
	- datatype of the parameter (**float**, **int** or **bool**). **bool** values include strings with two possible values.
	- type of change to apply to the paramter when it is modified (**add** or **ratio**).
	- default value of the parameter, to use in the first iteration
	- minimum value
	- maximum value
	- Maximum change to the parameter per iteration (unused if **bool**)

In cases where one parameters maximum or minimum values are defined by another parameter (for example, the bottom of an object being no higher than the top) the two parameters should be entered in sequence, and the max/min value should be entered as **prev**.

###staticParameters.txt

This file defines tracker parameters which are contained in the configuration files but should not be modified/optimized. Example:

*0,video-filename = test.mp4*
*0,database-filename = nyopt.sqlite*

The first value defined the configuration file in which to write the parameter; the second is simply the string to be written, in its entirety.

## 3 - Converting tracker output.

In order to calculate MOTA/MOTP, the tracker's output must be converted to the algorithm's format by the extract_trajectories function, located at the top of the code (TrOPed.py). The output must be an array of arrays, as below:

[
[objectid0,frame0,X0,Y0],
[objectid0,frame1,X1,Y1],
[objectid0,frame2,X2,Y2],
...]

Each record represents a single detection, defined as:

**objectid**: ID of the object, as identified by the tracker [int]

**frame**: Frame the object was detected at the following coordinates [int]

**X**, **Y**: (x,y) coordinates of the object at the given frame. In meters/world-space if homography was used, otherwise in pixels.
