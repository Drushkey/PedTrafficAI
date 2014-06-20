PEDESTRIAN TRAFFIC AI
- Dariush Ettehadieh, Polytechnique Montreal

The aim of this project is to calibrate TrafficIntelligence (https://bitbucket.org/Nicolas/trafficintelligence/wiki/Home) via simulated annealing for pedestrian tracking in a given video. For now, this process relies on manually traced trajectories: an excellent tool for these traces is JP Jodoin's annotation tool at www.jpjodoin.com/urbantracker/tools.html .

Version history:
	V1.01: Multiple changes:
- neighbor-solution finder now changes between 1 and 3 parameters per iteration, regardless of temperature
- parameters are modified in small increments, not randomly within their range
	- defined preliminary increments for all parameters
- new best solutions are now saved as [first 7 characters of working sqlite file]+best.sqlite
- algorithm now moves to worse solutions depending on temperature and delta_energy, in accordance with textbook simulated annealing
	V1.00: Build stabilized. Added option to optimize only grouping parameters.
	V0.04: Optimization loop closed.
	V0.03: Function: point_corresp_mod. Computes homography using cvutils, using points interpolated between extremes given my point_correspondence. Input is array of ratios (e.g. [0.5, 0.4, 0.6, 0.5]).
	V0.02: Point-correspondence: interpolation between ground and 1.5m vertical
	V0.01: Function: config_mod. Modifies the configuration file, inserting current parameters to be tested.
