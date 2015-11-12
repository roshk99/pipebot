#!/usr/bin/env python

#import rospy
import numpy as np
import random
import math
from scipy import interpolate

initial_guess = [np.array([-0.08, 0.05]), np.array([0.08, 0.05]), np.array([-0.08, 0.08]), np.array([0.08, 0.08]),np.array([0.00, 0.020]),np.array([-0.010, 0.020]),np.array([0.010, 0.020]) ]
K = 7
MIN_CLUSTER_SIZE = 3
SMOOTH = True
OUTLIER_REMOVAL = True
OUTLIER_MAX_DIST = 8
VERT_TOL = 1
VERT_SLOPE = 8
SPLINE_ORDER = 5
SMOOTHING_DEGREE = 0.5
LEFT_ANGLE1 = 155
LEFT_ANGLE2 = 105
RIGHT_ANGLE1 = 75
RIGHT_ANGLE2 = 25

class Cluster:
	def __init__(self, id_val, x, y):
		self.id = id_val
		self.mu = [x,y]
	def __str__(self):
		s = "Id: {0}, Mean: [{1}, {2}]".format(self.id, self.mu[0], self.mu[1])
		return s
	def get_id(self):
		return self.id
	def get_mu(self):
		return list(self.mu)
	def update_mean(self, x, y):
		self.mu = [x,y]


class DataPoint:
	def __init__(self, id_val, x, y, angle):
		self.id = id_val
		self.coordinates = [x, y]
		self.cluster = None
		self.angle = angle
		self.slope = None
		self.smoothed_coordinates = [None, None]
	def __str__(self):
		s = "Id: {0}, Angle: {4}, Coordinates: [{1}, {2}], Cluster_Id: {3}, Slope: {5}".format(self.id, self.coordinates[0], self.coordinates[1], self.cluster, self.angle, self.slope)
		return s
	def get_id(self):
		return self.id
	def get_coordinates(self):
		return list(self.coordinates)
	def get_cluster(self):
		return self.cluster
	def set_cluster(self, cluster):
		self.cluster = cluster
	def get_angle(self):
		return self.angle
	def add_slope(self, slope):
		self.slope = slope
	def add_smoothed_coordinates(self, x, y):
		self.smoothed_coordinates = [x,y]
	def get_smoothed_coordinates(self):
		return list(self.smoothed_coordinates)
	def get_slope(self):
		return self.slope


class Clusters:
	def __init__(self):
		self.clusters = {}
	def __str__(self):
		s = ""
		for cluster in self.clusters.values():
			s = "{0}, [{1}]".format(s, str(cluster))
		return s
	def cluster_added(self, cluster_id):
		return cluster_id in self.clusters.keys()
	def add_cluster(self, cluster):
		if cluster.get_id() not in self.clusters.keys():
			self.clusters[cluster.get_id()] = cluster
		else:
			print 'Replacing a Cluster'
			self.clusters[cluster.get_id()] = cluster
	def get_clusters(self):
		return list(self.clusters.values())
	def formatted_clusters(self):
		s = ""
		for cluster in self.clusters.values():
			mu = cluster.get_mu()
			s = "{0}, [{1}, {2}]".format(s, mu[0], mu[1])
		return s
	def get_cluster_ids(self):
		return list(self.clusters.keys())
	def remove_cluster(self, cluster_id):
		self.clusters.pop(cluster_id)
	def get_cluster(self, cluster_id):
		return self.clusters[cluster_id]


class DataPoints:
	def __init__(self):
		self.points = {}
	def __str__(self):
		s = ""
		for point in self.points.values():
			s = "{0}, [{1}]".format(s, str(point))
		return s
	def get_points_array(self):
		x = []
		y = []
		for point in self.points.values():
			coordinates = point.get_coordinates()
			x.append(coordinates[0])
			y.append(coordinates[1])
		point_arr = np.concatenate((np.array([x]), np.array([y])), axis=0).T
		return point_arr
	def add_point(self, point):
		if point.get_id() not in self.points.keys():
			self.points[point.get_id()] = point
		else:
			print 'Replacing a Point'
			self.points[point.get_id()] = point
	def remove_point(self, point):
		if point.get_id in self.points.keys():
			self.points.pop(point.get_id())
	def get_points(self):
		return list(self.points.values())
	def find_cluster(self, point_id, mu):
		point = self.points[point_id]
		coordinates = np.array(point.get_coordinates())
	 	mu_key_arr = []
	 	for i in enumerate(mu):
	 		if mu[i[0]][0] is not None:
	 			mu_key_arr.append(np.linalg.norm(coordinates-mu[i[0]]))
 			else:
 				mu_key_arr.append(None)
		bestmukey = np.argmin(np.array(mu_key_arr))
	 	return bestmukey
 	def get_cluster_points(self, cluster_id):
 		cluster_points = []
 		for point in self.points.values():
 			if point.get_cluster() == cluster_id:
 				cluster_points.append(point)
		return cluster_points
	def cluster_points_str(self, cluster_id):
		cluster_points = self.get_cluster_points(cluster_id)
		s = ""
		for point in cluster_points:
			coordinates = point.get_coordinates()
			s = "{0}, [{1},{2}]".format(s, coordinates[0], coordinates[1])
		return s
	def remove_cluster(self, cluster_id):
		cluster_points = self.get_cluster_points(cluster_id)
		for point in cluster_points:
			self.remove_point(point)
	def add_smoothed_coordinates(self, point_id, x, y):
		point = self.points[point_id]
		point.add_smoothed_coordinates(x, y)
	def get_ordered_points(self):
		points = []
		for key in sorted(self.points.keys()):
			points.append(self.points[key])
		return points
	def add_slope(self, point_id, slope):
		point = self.points[point_id]
		point.add_slope(slope)
	def assign_slopes(self, cluster_id, spline_order=3, smoothing_degree=0.3):
		cluster_points = self.get_cluster_points(cluster_id)
		x_raw = []
		y_raw = []
		id_raw = []
		for point in cluster_points:
			coordinates = point.get_coordinates()
			x_raw.append(coordinates[0])
			y_raw.append(coordinates[1])
			id_raw.append(point.get_id())
		mu_x = sum(x_raw)/float(len(x_raw))
		mu_y = sum(y_raw)/float(len(y_raw))
		
		#Sort
		x_raw = np.array(x_raw)
		y_raw = np.array(y_raw)
		id_raw = np.array(id_raw)
		sorted_idxs = np.argsort(x_raw)
		x_raw = x_raw[sorted_idxs]
		y_raw = y_raw[sorted_idxs]
		id_raw = id_raw[sorted_idxs]

		tck = interpolate.splrep(x_raw, y_raw, k=min(spline_order, len(x_raw)-1), s=smoothing_degree)
		x_new = x_raw
		y_new = interpolate.splev(x_new, tck)
		y_new_der = interpolate.splev(x_new, tck, der=1)

		for ii in range(len(id_raw)):
			self.add_slope(id_raw[ii], y_new_der[ii])
			self.add_smoothed_coordinates(id_raw[ii], x_new[ii], y_new[ii])


def deg_to_rad(deg):
	return deg*math.pi/180.0


def rad_to_deg(rad):
	return rad*180.0/math.pi


def find_cluster_mean(points):
	x = []
	y = []
	for point in points:
		coordinates = point.get_coordinates()
		x.append(coordinates[0])
		y.append(coordinates[1])
	return np.mean(x), np.mean(y)


def cluster_points(data_points, clusters, mu):
	for point in data_points.get_points():
		cluster_id = data_points.find_cluster(point.get_id(), mu)
		if not clusters.cluster_added(cluster_id) and not cluster_id == None:
			cluster = Cluster(cluster_id, mu[cluster_id][0], mu[cluster_id][1])
			clusters.add_cluster(cluster)
		point.set_cluster(cluster_id)
	return clusters


def reevaluate_centers(data_points, clusters, mu):
	newmu = []
	for cluster in clusters.get_clusters():
		cluster_points = data_points.get_cluster_points(cluster.get_id())
		if len(cluster_points) > 0:
			mu_x, mu_y = find_cluster_mean(cluster_points)
			cluster.update_mean(mu_x, mu_y)
			newmu.append(np.array([mu_x, mu_y]))
		else:
			newmu.append(np.array([None, None]))

	return newmu
 
def has_converged(mu, oldmu):
    list1 = []
    list2 = []
    for a1, a2 in zip(mu, oldmu):
        list1.append(tuple(a1))
        list2.append(tuple(a2))
    return set(list1) == set(list2)


def find_centers(data_points, clusters, K):
	X = data_points.get_points_array()

	# Initialize to K random centers
	oldmu = random.sample(X, K)
	mu = initial_guess
	while not has_converged(mu, oldmu):
		oldmu = mu
		# Assign all points in X to clusters
		cluster_points(data_points, clusters, mu)
		# Reevaluate centers
		mu = reevaluate_centers(data_points, clusters, oldmu)
	return data_points, clusters


def process_data(data_points, clusters):
	for cluster_id in clusters.get_cluster_ids():
		cluster_points = data_points.get_cluster_points(cluster_id)
		if len(cluster_points) < MIN_CLUSTER_SIZE:
			data_points.remove_cluster(cluster_id)
			clusters.remove_cluster(cluster_id)
		else:
			data_points.assign_slopes(cluster_id, SPLINE_ORDER, SMOOTHING_DEGREE)


def get_slope_vec(data_points):
	slopes = {}
	for point in data_points.get_points():
		slope = point.get_slope()
		angle = point.get_angle()
		slopes[angle] = slope
	return slopes


def is_vertical(slope):
	return abs(slope) + VERT_TOL > VERT_SLOPE


def coarse_classification(data_points, clusters):
	slopes = get_slope_vec(data_points)
	junction_right = False
	junction_left = False

	for angle, slope in slopes.items():
		if slope:
			if angle < deg_to_rad(RIGHT_ANGLE1) and angle > deg_to_rad(RIGHT_ANGLE2):
				if not is_vertical(slope):
					junction_right = True
			elif angle < deg_to_rad(LEFT_ANGLE1) and angle > deg_to_rad(LEFT_ANGLE2):
				if not is_vertical(slope):
					junction_left = True
			#print angle*180/math.pi, slope
	return [junction_left, junction_right]


def generate_matlab_plot(data_points, clusters):
	x = []
	y = []
	x_smooth = []
	y_smooth = []
	cluster_x = []
	cluster_y = []
	for point in data_points.get_points():
		coordinates = point.get_coordinates()
		smoothed_coordinates = point.get_smoothed_coordinates()
		x.append(coordinates[0])
		y.append(coordinates[1])
		x_smooth.append(smoothed_coordinates[0])
		y_smooth.append(smoothed_coordinates[1])
	for cluster in clusters.get_clusters():
		coordinates = cluster.get_mu()
		cluster_x.append(coordinates[0])
		cluster_y.append(coordinates[1])
	#print 'x=', x, ';y=', y, ';x_smooth=', x_smooth, ';y_smooth=', y_smooth, ";plot(x,y,'.b', 'MarkerSize', 20);hold on; plot(x_smooth,y_smooth, 'r');hold off; axis square; axis equal;"
	print 'x=', x, ';y=', y, ';x1=', cluster_x, ';y1=', cluster_y, ";plot(x,y,'.b', 'MarkerSize', 20);hold on; plot(x1,y1, 'r.', 'MarkerSize', 30);hold off; axis square; axis equal;"


def pre_fine_classification(data_points, clusters):
	angles_left = []
	slopes_left = []
	for point in data_points.get_points():
		angle = point.get_angle()
		slope = point.get_slope()
		if angle < deg_to_rad(LEFT_ANGLE1) and angle > deg_to_rad(LEFT_ANGLE2):
			angles_left.append(rad_to_deg(angle))
			slopes_left.append(slope)
	idxs = np.argsort(angles_left)
	angles_left = list(np.array(angles_left)[idxs])
	slopes_left = list(np.array(slopes_left)[idxs])
	
	angles_right = []
	slopes_right = []
	for point in data_points.get_points():
		angle = point.get_angle()
		slope = point.get_slope()
		if angle < deg_to_rad(RIGHT_ANGLE1) and angle > deg_to_rad(RIGHT_ANGLE2):
			angles_right.append(rad_to_deg(angle))
			slopes_right.append(slope)
	idxs = np.argsort(angles_right)
	angles_right = list(np.array(angles_right)[idxs])
	slopes_right = list(np.array(slopes_right)[idxs])
	
	print angles_left, slopes_left
	print angles_right, slopes_right
	return [angles_left, angles_right], [slopes_left, slopes_right]

def fine_classification_left(data_points, clusters):
	result = pre_fine_classification(data_points, clusters)


def fine_classification_right(data_points, clusters):
	result = pre_fine_classification(data_points, clusters)


def fine_classification_both(data_points, clusters):
	result = pre_fine_classification(data_points, clusters)


def algorithm(x, y, plot=False, debug=False):
	data_points = DataPoints()
	clusters = Clusters()
	# i = 0
	# for point in data:
	# 	if point.x == 0.0:
	# 		angle = math.pi/2
	# 	else:
	# 		angle = math.atan(point.y / point.x)
	# 	point = DataPoint(i, point.x, point.y, deg_to_rad(angle))
	# 	data_points.add_point(point)
	# 	i += 1
	
	i = 0
	for xval, yval in zip(x,y):
		if xval == 0.0:
			angle = math.pi/2
		else:
			angle = math.atan2(yval,xval)
			if angle < 0: angle += 2*math.pi
		point = DataPoint(i, xval, yval, angle)
		data_points.add_point(point)
		i += 1
	find_centers(data_points, clusters, K)
	print clusters
	# process_data(data_points, clusters)
	# coarse_junction = coarse_classification(data_points, clusters)
	
	# if coarse_junction[0] and not coarse_junction[1]:
	# 	fine_classification_left(data_points, clusters)
	# elif not coarse_junction[0] and coarse_junction[1]:
	# 	fine_classification_right(data_points, clusters)
	# else:
	# 	fine_classification_both(data_points, clusters)
	if plot:
		generate_matlab_plot(data_points, clusters)
	# return coarse_junction
	
def main(data):
	rospy.loginfo("Processed Data First Point: " + str(data.points[0].x) + "," + str(data.points[0].y))
	#result = algorithm(data.points)
	#print 'result', result

    


