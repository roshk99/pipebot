import numpy as np
import random
import math
#import matplotlib.pyplot as plt
from scipy import interpolate

initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 8]), np.array([8, 8]),np.array([0, 20]),np.array([-10, 20]),np.array([10, 20]) ]
TOL1 = 2.4 #vertical slope
TOL2 = 0.5 #horizontal slope
TOL3 = 0.3 #max dist
VERT_SLOPE = 5
DIST_TOL = 8 #outlier identification
MAX_DIST = 40
K = len(initial_guess)

def cluster_points(X, mu):
    clusters  = {}
    labels = []
    for x in X:
        bestmukey = min([(i[0], np.linalg.norm(x-mu[i[0]])) \
                    for i in enumerate(mu)], key=lambda t:t[1])[0]
        try:
            clusters[bestmukey].append(x)
        except KeyError:
            clusters[bestmukey] = [x]
        labels.append(bestmukey)
    return clusters, labels
 
def reevaluate_centers(mu, clusters):
    newmu = []
    keys = sorted(clusters.keys())
    for k in keys:
        newmu.append(np.mean(clusters[k], axis = 0))
    return newmu
 
def has_converged(mu, oldmu):
    list1 = []
    list2 = []
    for a1, a2 in zip(mu, oldmu):
        list1.append(tuple(a1))
        list2.append(tuple(a2))
    return set(list1) == set(list2)

def find_centers(X, K):
    # Initialize to K random centers
    oldmu = random.sample(X, K)
    mu = initial_guess
    while not has_converged(mu, oldmu):
        oldmu = mu
        # Assign all points in X to clusters
        clusters, labels = cluster_points(X, mu)
        # Reevaluate centers
        mu = reevaluate_centers(oldmu, clusters)
    return(mu, clusters, labels)

def deg_to_rad(angle):
    return math.pi*angle/180.0

def plot_data(data):
    colors=[255,0,0], [255,127,0], [0,255,0], [0,255,255], [0,0,255],[127,0,255], [255,0,255]

    print 'FOR MATLAB'
    print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
    print 'close all; clc;'
    print 'mu_x=', data[0], '; mu_y=', data[1], "; plot(mu_x, mu_y, 'k.', 'MarkerSize', 30); hold on;"
    for i in range(len(data[2])):
        print "plot(", data[2][i], ",", data[3][i], ", '.', 'Color', ", colors[i], "./255, 'MarkerSize', 20);"
        print "plot(", data[4][i], ",", data[5][i], ",'Color', ", colors[i], "./255, 'Linewidth', 2);"
    print "hold off;axis square; axis equal;"

    print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'

def is_vertical(slope):
    return abs(slope) + TOL1 > VERT_SLOPE
def is_horizontal(slope):
    return abs(slope) < TOL2
def is_max_dist(x,y):
    return math.sqrt(x**2 + y**2) >= MAX_DIST - TOL3

def get_cluster_num_map(labels, clusters):
    num_points = []
    for i in range(len(clusters)):
        num_points.append(len(clusters[i]))

    mapping = {}
    order = 0
    for label in labels:
        if label not in mapping.keys() and num_points[label] > 2:
            mapping[label] = order
            order +=1
    reversed_mapping = {}
    total_num = len(mapping.keys())
    for key, value in mapping.items():
        reversed_mapping[key] = total_num - value - 1

    return reversed_mapping

def process_data(x_raw, y_raw):
    mu_x = sum(x_raw)/float(len(x_raw))
    mu_y = sum(y_raw)/float(len(y_raw))
    xd = []
    yd = []
    for x,y in zip(x_raw, y_raw):
        dist = math.sqrt((x-mu_x)**2 + (y-mu_y)**2)
        if dist < DIST_TOL:
            xd.append(x)
            yd.append(y)
    t = range(len(xd))
    xd = list(interpolate.UnivariateSpline(t, xd, k=1)(t))
    yd = list(interpolate.UnivariateSpline(t, yd, k=1)(t))
    #yd = list(interpolate.UnivariateSpline(xd, yd, k=1)(xd))
    return xd, yd

def stage_1(mu, clusters, labels, print_bool):

    #Match cluster number and left-to-right order
    cluster_num_map = get_cluster_num_map(labels, clusters)

    #Initialize arrays
    mus_x = []
    mus_y = []
    clusters_xd = []
    clusters_yd = []
    clusters_xl = []
    clusters_yl = []
    slopes = {}

    #For each cluster
    for i in range(len(mu)):
        #Check if number of points in cluster is more than 2
        num_points = len(clusters[i])
        if num_points > 2:

            #Smooth individual x and y data linearly
            xd = []
            yd = []
            for arr in clusters[i]:
                xd.append(arr[0])
                yd.append(arr[1])
            xd, yd = process_data(xd, yd)

            #Get the slope for the cluster
            A = np.vstack([xd, np.ones(len(xd))]).T
            slope, intercept = np.linalg.lstsq(A,yd)[0]

            #Append slope in appropriate place
            slopes[cluster_num_map[i]] = slope

            #Create vectors for plotting
            xl = list(np.linspace(min(xd), max(xd), 30))
            yl = [slope*xx + intercept for xx in xl]
            yl2 = list(np.linspace(min(yd), max(yd), 30))
            xl2 = [(yy - intercept)/slope for yy in yl2]
            mu_x = mu[i][0]
            mu_y = mu[i][1]

            #Store Values
            clusters_xd.append(xd)
            clusters_yd.append(yd)
            clusters_xl.append(xl + xl2)
            clusters_yl.append(yl + yl2)
            mus_x.append(np.mean(xd))
            mus_y.append(np.mean(yd))

    #Get number of clusters used
    cluster_num = len(clusters_xd)
    if print_bool:
        print 'Clusters Used:', cluster_num

    if cluster_num not in [4, 5, 6, 7]:
        print 'Error occurred'
        print 'Only', cluster_num, ' clusters created'
        return False
    
    if print_bool:
        print 'Slopes:', slopes

    junction_left = False
    junction_right = False
    if cluster_num == 7:
        #Check Left Wall straight
        if not is_vertical(slopes[0]) or not is_vertical(slopes[1]):
            junction_left = True
            if print_bool: print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(slopes[5]) or not is_vertical(slopes[6]):
            junction_right = True
            if print_bool: print 'Rule2'
        #Check Front not straight
        if not is_max_dist(mus_x[4], mus_y[4]) and is_horizontal(slopes[4]):
            junction_left = True
            junction_right = True
            if print_bool: print 'Rule3'
    if cluster_num == 5:
        #Check Left Wall straight
        if not is_vertical(slopes[0]) or not is_vertical(slopes[1]):
            junction_left = True
            if print_bool: print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(slopes[3]) or not is_vertical(slopes[4]):
            junction_right = True
            if print_bool: print 'Rule2'
        #Check Front not straight
        if not is_max_dist(mus_x[2], mus_y[2])and is_horizontal(slopes[2]):
            junction_left = True
            junction_right = True
            if print_bool: print 'Rule3'
    if cluster_num == 6:
        #Fixes unknown error
        slope_right = slopes[4]
        slope_left = slopes[5]
        slopes[4] = slope_left
        slopes[5] = slope_right

        #Check Left Wall straight
        if not is_vertical(slopes[0]) or not is_vertical(slopes[1]):
            junction_left = True
            if print_bool: print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(slopes[5]) or not is_vertical(slopes[4]):
            junction_right = True
            if print_bool: print 'Rule2'
        #Check Front not straight
        if (not is_max_dist(mus_x[2], mus_y[2]) and is_horizontal(slopes[2])) or (not is_max_dist(mus_x[3], mus_y[3]) and is_horizontal(slopes[3])):
            junction_left = True
            junction_right = True
            if print_bool: print 'Rule3'
    if cluster_num == 4:
        #Check Left Wall straight
        if not is_vertical(slopes[0]):
            junction_left = True
            if print_bool: print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(slopes[3]):
            junction_right = True
            if print_bool: print 'Rule2'
        #Check Front not straight
        if (not is_max_dist(mus_x[2], mus_y[2]) and is_horizontal(slopes[2])) or (not is_max_dist(mus_x[1], mus_y[1]) and is_horizontal(slopes[1])):
            junction_left = True
            junction_right = True
            if print_bool: print 'Rule3'
    return ([mus_x, mus_y, clusters_xd, clusters_yd, clusters_xl, clusters_yl, slopes], [junction_left, junction_right])

def algorithm(data, plot_bool, print_bool):
    x = []
    y = []
    #angles = []
    #distances = []
    for angle, distance in data:
        #angles.append(deg_to_rad(angle))
        #distances.append(distance)
        x.append(distance*math.cos(deg_to_rad(angle)))
        y.append(distance*math.sin(deg_to_rad(angle)))
    X = np.concatenate((np.array([x]), np.array([y])), axis=0).T

    mu, clusters, labels = find_centers(X,K)
    print clusters
    result = stage_1(mu, clusters, labels, print_bool)
    if result:
        if plot_bool:
            plot_data(result[0])
        return result[1]
    #else:
    #print 'x=', angles, ';y=', distances, ";polar(x,y,'+');"

data1 = [(25.000000000000007, 21.76352273759297), (22.562465969218362, 11.42489340644706), (25.864053893396175, 11.649640327151243), (31.741550877148207, 11.82402254564993), (32.84103544265468, 12.475393675272098), (36.608966814751746, 13.27764978505417), (40.704180759149374, 14.739845321034608), (51.77578008136326, 23.28146804696025), (54.4953505632501, 22.392901866126156), (56.25770732314708, 22.33973357122185), (58.256505347050016, 21.718892298551136), (60.95900714737005, 21.017591453313532), (62.98871796901191, 20.48236485637605), (65.62125585729198, 20.440359434177196), (68.02794347127245, 20.349469741329916), (70.55024150253766, 19.734022814485968), (73.07143134517946, 19.697062261867323), (75.92635308011357, 19.650066237332908), (78.83059656310863, 19.560922618820136), (81.20787155513108, 19.552845327117613), (84.6207185579935, 19.446501517792083), (87.49553348164835, 19.411576129782063), (90.09980444670254, 19.355722990338116), (93.21052808945679, 18.803828367427858), (100.92301378716529, 21.416348559008494), (103.80680708151448, 20.991189647621336), (106.45239479870995, 19.74030337932599), (109.10245178376282, 17.754987694330442), (111.31129863008768, 14.820851758574163), (114.06833639647449, 14.0055140782041), (116.77722283510786, 13.349020430046522), (119.62495402960238, 12.768035253890954), (122.24794884776072, 12.091392876351891), (124.87110021774033, 11.416847369505003), (128.19856056388394, 11.548437189984368), (131.94982168606657, 11.298095993248547), (134.5166541568955, 10.570733505761412), (136.50943619614878, 10.024902324356425), (139.90155849559108, 8.97287191660959), (141.66422617235682, 8.148026872195027), (147.14490676836715, 7.604756971873976), (149.80074720626996, 7.326795782319472), (154.6948977254968, 6.949627109519094), (154.0, 6.683042369128452)]
algorithm(data1, False, False)
