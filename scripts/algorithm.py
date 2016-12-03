import numpy as np
import random
import math
#import matplotlib.pyplot as plt
from scipy import interpolate

initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 8]), np.array([8, 8]),np.array([0, 20]),np.array([-10, 20]),np.array([10, 20]) ]
#initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 8]), np.array([8, 8]), np.array([0, 20])]
TOL1 = 3
TOL2 = 1
TOL3 = 5
TOL4 = 0.5 #vertical slope
TOL4 = 0.3 #horizontal slope
TOL5 = 0.5 #equality
VERT_SLOPE = 1.5
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

def sort_left_to_right(labels, clusters_to_use):
    sorted_ind = []
    for label in labels:
        if label not in sorted_ind and label in clusters_to_use:
            sorted_ind.append(label)
    sorted_ind = list(reversed(sorted_ind))
    return sorted_ind

def is_vertical(slope):
    return slope - VERT_SLOPE - TOL3 > 0
def is_horizontal(slope):
    return abs(slope) < TOL4
def is_parallel(slope1, slope2):
    return abs(slope1 - slope2) < TOL5

def stage_1(mu, clusters, labels):
    #Create numpy array
    mu_x = []
    mu_y = []
    for arr in mu:
        mu_x.append(arr[0])
        mu_y.append(arr[1])
    mu_arr = np.array([mu_x, mu_y])

    junction_left = False
    junction_right = False
    
    clusters_x = []
    clusters_y = []
    clusters_xl = []
    clusters_yl = []
    slopes = []
    intercepts = []
    clusters_to_use = []
    for cluster_num, cluster_values in clusters.items():
        xd = []
        yd = []
        for arr in cluster_values:
            xd.append(arr[0])
            yd.append(arr[1])
        if len(xd) > 2:
            t = range(len(xd))
            xd = list(interpolate.UnivariateSpline(t, xd, k=1)(t))
            yd = list(interpolate.UnivariateSpline(t, yd, k=1)(t))
            
            #f = interpolate.interp1d(xd, yd, kind='linear')
            #ynew = f(xd)
            #yd = list(ynew)

            A = np.vstack([xd, np.ones(len(xd))]).T
            slope, intercept = np.linalg.lstsq(A,yd)[0]

            xl = list(np.linspace(min(xd), max(xd), 30))
            yl = [slope*xx + intercept for xx in xl]
            
            yl2 = list(np.linspace(min(yd), max(yd), 30))
            xl2 = [(yy - intercept)/slope for yy in yl2]
            #print min(xd), max(xd), min(yd), max(yd)

            clusters_x.append(xd)
            clusters_y.append(yd)
            #clusters_xl.append(xl + xl2)
            #clusters_yl.append(yl + yl2)
            clusters_xl.append(xl)
            clusters_yl.append(yl)
            slopes.append(slope)
            intercepts.append(intercept)
            clusters_to_use.append(cluster_num)

    cluster_num = len(slopes)
    sorted_ind = sort_left_to_right(labels, clusters_to_use)
    print len(slopes), len(sorted_ind)
    #print 'SLOPES', slopes
    #print 'sorted_indx', sorted_ind
    sorted_slopes = []
    for j in range(len(slopes)):
        sorted_slopes.append(slopes[sorted_ind[j]])
    #print 'Sorted Slopes', sorted_slopes

    #print 'Clusters Used:', cluster_num

    if cluster_num not in [5, 6, 7]:
        print 'Error occurred'
        print 'Only', cluster_num, ' clusters created'
        return False
    
    if cluster_num == 7:
        #Check Left Wall straight
        if not is_vertical(sorted_slopes[1]) or not is_vertical(sorted_slopes[2]):
            junction_left = True
            #print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(sorted_slopes[4]) or not is_vertical(sorted_slopes[5]):
            junction_right = True
            #print 'Rule2'
        #Check Front not straight
        if is_horizontal(sorted_slopes[3]) and is_horizontal(sorted_slopes[4]):
            junction_left = True
            junction_right = True
            #print 'Rule3'
    if cluster_num == 5:
        #Check Left Wall straight
        if not is_vertical(sorted_slopes[0]) or not is_vertical(sorted_slopes[1]):
            junction_left = True
            #print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(sorted_slopes[3]) or not is_vertical(sorted_slopes[4]):
            junction_right = True
            #print 'Rule2'
        #Check Front not straight
        if is_horizontal(sorted_slopes[1]) and is_horizontal(sorted_slopes[3]):
            junction_left = True
            junction_right = True
            #print 'Rule3'
    if cluster_num == 6:
        #Check Left Wall straight
        if not is_vertical(sorted_slopes[0]) or not is_vertical(sorted_slopes[1]):
            junction_left = True
            #print 'Rule1'
        #Check Right Wall straight
        if not is_vertical(sorted_slopes[4]) or not is_vertical(sorted_slopes[5]):
            junction_right = True
            #print 'Rule2'
        #Check Front not straight
        if is_horizontal(sorted_slopes[2]) and is_horizontal(sorted_slopes[3]):
            junction_left = True
            junction_right = True
            #print 'Rule3'

        
    return ([mu_x, mu_y, clusters_x, clusters_y, clusters_xl, clusters_yl, slopes, intercepts], [junction_left, junction_right])

def algorithm(data, plot_bool):
    x = []
    y = []
    angles = []
    distances = []
    for angle, distance in data:
        angles.append(deg_to_rad(angle))
        distances.append(distance)
        x.append(distance*math.cos(deg_to_rad(angle)))
        y.append(distance*math.sin(deg_to_rad(angle)))
    X = np.concatenate((np.array([x]), np.array([y])), axis=0).T

    mu, clusters, labels = find_centers(X,K)
    result = stage_1(mu, clusters, labels)
    if result:
        if plot_bool:
            plot_data(result[0])
        return result[1]
    else:
        print 'x=', angles, ';y=', distances, ";polar(x,y,'+');"