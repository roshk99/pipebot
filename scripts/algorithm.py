import numpy as np
import random
import math

#import matplotlib.pyplot as plt

initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 8]), np.array([8, 8]),np.array([0, 20]),np.array([-10, 20]),np.array([10, 20]) ]
#initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 8]), np.array([8, 8]), np.array([0, 20])]
TOL1 = 3
TOL2 = 1
TOL3 = 0.3
TOL4 = 5
VERT_SLOPE = 10
K = len(initial_guess)

def cluster_points(X, mu):
    clusters  = {}
    for x in X:
        bestmukey = min([(i[0], np.linalg.norm(x-mu[i[0]])) \
                    for i in enumerate(mu)], key=lambda t:t[1])[0]
        try:
            clusters[bestmukey].append(x)
        except KeyError:
            clusters[bestmukey] = [x]
    return clusters
 
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
        clusters = cluster_points(X, mu)
        # Reevaluate centers
        mu = reevaluate_centers(oldmu, clusters)
    return(mu, clusters)

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

def stage_1(mu, clusters):
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
    for cluster_num, cluster_values in clusters.items():
        xd = []
        yd = []
        for arr in cluster_values:
            xd.append(arr[0])
            yd.append(arr[1])
        
        A = np.vstack([xd, np.ones(len(xd))]).T
        slope, intercept = np.linalg.lstsq(A,yd)[0]

        xl = list(np.linspace(min(xd), max(xd), 30))
        yl = [slope*xx + intercept for xx in xl]
        
        yl2 = list(np.linspace(min(yd), max(yd), 30))
        xl2 = [(yy - intercept)/slope for yy in yl2]

        clusters_x.append(xd)
        clusters_y.append(yd)
        clusters_xl.append(xl + xl2)
        clusters_yl.append(yl + yl2)
        slopes.append(slope)
        intercepts.append(intercept)
        
    #Split into left, right, middle
    if len(slopes) == 7:
        sorted_ind = np.argsort(np.array(mu_x))
        left_ind = sorted_ind[0:3]
        mid_ind = sorted_ind[3]
        right_ind = sorted_ind[4:7]
    
        #Analyze slopes
    
        #Check for T-junction
        if abs(slopes[left_ind[2]]) < TOL3 and abs(slopes[mid_ind]) < TOL3 and abs(slopes[right_ind[0]]) < TOL3:
            junction_left = True
            junction_right = True
    
        #Check if left wall is straight
        if abs(slopes[left_ind[1]] - VERT_SLOPE) > TOL4 or abs(slopes[left_ind[2]] - VERT_SLOPE) < TOL4:
            junction_left = True
            junction_right = False
            
        #Check if right wall is straight
        if abs(slopes[right_ind[1]] - VERT_SLOPE) > TOL4 or abs(slopes[right_ind[2]] - VERT_SLOPE) < TOL4:
            junction_right = True
            junction_left = False
            
    elif len(slopes) == 5:
        sorted_ind = np.argsort(np.array(mu_x))
        left_ind = sorted_ind[0:2]
        mid_ind = sorted_ind[2]
        right_ind = sorted_ind[3:5]
        #Analyze slopes
    
        #Check for T-junction
        if abs(slopes[left_ind[1]]) < TOL3 and abs(slopes[mid_ind]) < TOL3 and abs(slopes[right_ind[0]]) < TOL3:
            junction_left = True
            junction_right = True
    
        #Check if left wall is straight
        if abs(slopes[left_ind[0]] - VERT_SLOPE) > TOL4 or abs(slopes[left_ind[0]] - VERT_SLOPE) < TOL4:
            junction_left = True
            junction_right = False
            
        #Check if right wall is straight
        if abs(slopes[right_ind[0]] - VERT_SLOPE) > TOL4 or abs(slopes[right_ind[1]] - VERT_SLOPE) < TOL4:
            junction_right = True
            junction_left = False
            
    elif len(slopes) == 6:
        sorted_ind = np.argsort(np.array(mu_x))
        left_ind = sorted_ind[0:2]
        mid_ind = sorted_ind[2]
        right_ind = sorted_ind[4:6]
        #Analyze slopes
    
        #Check for T-junction
        if abs(slopes[left_ind[1]]) < TOL3 and abs(slopes[mid_ind]) < TOL3 and abs(slopes[right_ind[0]]) < TOL3:
            junction_left = True
            junction_right = True
    
        #Check if left wall is straight
        if abs(slopes[left_ind[0]] - VERT_SLOPE) > TOL4 or abs(slopes[left_ind[0]] - VERT_SLOPE) < TOL4:
            junction_left = True
            junction_right = False
    
        #Check if right wall is straight
        if abs(slopes[right_ind[0]] - VERT_SLOPE) > TOL4 or abs(slopes[right_ind[1]] - VERT_SLOPE) < TOL4:
            junction_right = True
            junction_left = False
            
    else:
        print 'Error occurred'
        print 'Only', len(slopes), ' clusters created'
        return False
        
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

    mu, clusters = find_centers(X,K)
    
    result = stage_1(mu, clusters)
    if result:
        if plot_bool:
            plot_data(result[0])
        return result[1]
    else:
        print 'x=', angles, ';y=', distances, ";polar(x,y,'+');"
