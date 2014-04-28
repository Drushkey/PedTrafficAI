PEDESTRIAN TRAFFIC AI
- Dariush Ettehadieh, Polytechnique Montreal

The aim of this project is to calibrate TrafficIntelligence (https://bitbucket.org/Nicolas/trafficintelligence/wiki/Home) via simulated annealing for pedestrian tracking in a given video. For now, this process relies on manually traced trajectories: an excellent tool for these traces is JP Jodoin's annotation tool at www.jpjodoin.com/urbantracker/tools.html .

Version history:
	V0.03: Function: point_corresp_mod. Computes homography using cvutils, using points interpolated between extremes given my point_correspondence. Input is array of ratios (e.g. [0.5, 0.4, 0.6, 0.5]).
	V0.02: Point-correspondence: interpolation between ground and 1.5m vertical
	V0.01: Function: config_mod. Modifies the configuration file, inserting current parameters to be tested.
