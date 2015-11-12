import numpy as np
import random
import math
#import matplotlib.pyplot as plt

initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 8]), np.array([8, 8]),np.array([0, 20]),np.array([-10, 20]),np.array([10, 20]) ]
TOL1 = 3
TOL2 = 1
K = 7

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
    print 'FOR MATLAB'
    print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
    print 'close all; clc; clear all;'
    print 'mu_x=', data[0], '; mu_y=', data[1], "; plot(mu_x, mu_y, 'k.', 'MarkerSize', 30); hold on;"
    print "colors={[255,0,0], [255,127,0], [255,255,0], [0,255,0], [0,255,255], [127,0,255], [255,0,255]};"
    for i in range(len(clusters_x)):
        print "for i=1:", len(data[2])
        print "    plot(", clusters_x[i], ",", clusters_y[i], ", '.', 'Color', colors{", i+1, "}, 'MarkerSize', 20);"
        print "    plot(", clusters_x1[i], ",", clusters_y1[i], ", Color', colors{", i+1, "}, 'Linewidth', 2);"
    print "hold off; axis square; axis equal;"

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
    clusters_x1 = []
    clusters_y1 = []
    
    for cluster_num, cluster_values in clusters.items():
        xd = []
        yd = []
        for arr in cluster_values:
            xd.append(arr[0])
            yd.append(arr[1])
        par = np.polyfit(xd, yd, 1, full=False)
        slope=par[0]
        intercept=par[1]
        xl = list(np.linspace(min(xd), max(xd)))
        yl = [slope*xx + intercept for xx in x_l]
        
        clusters_x.append(xd)
        clusters_y.append(yd)
        clusters_x1.append(x1)
        clusters_y1.append(y1)
    
    return ([mu_x, mu_y, clusters_x, clusters_y, clusters_x1, clusters_y1], [junction_left, junction_right])

def algorithm(data, plot_bool):
    x0 = data[0][1]*math.cos(deg_to_rad(data[0][0]))
    y0 = data[0][1]*math.sin(deg_to_rad(data[0][0]))
    X = np.array([[x0, y0]])
    for i in range(1, len(data)):
        x = data[i][1]*math.cos(deg_to_rad(data[i][0]))
        y = data[i][1]*math.sin(deg_to_rad(data[i][0]))
        X = np.concatenate((X, np.array([[x, y]])), axis=0)

    mu, clusters = find_centers(X,K)
    
    #result = stage_1(mu, clusters)
    # if result:
    #     print 'Mu', result[0]
    #     print 'Junction', result[1]
    
    #if plot_bool:
        #plot_data(result[0])
    
    # if result:
    #     return result[1]
    # else:
    #     return {}

data1 = [(25.0, 30), (30.0, 17.81178458266516), (35.0, 11.903305942310643), (40.0, 15.66182011922054), (45.0, 18.925790577325845), (50.0, 19.255034932107172), (55.0, 19.286351801651108), (60.0, 19.286351801651108), (65.0, 19.29419988284662), (70.0, 19.1617989930255), (75.0, 18.94833041111653), (80.0, 18.933296727468882), (85.0, 18.982259379286038), (90.0, 18.907055359206044), (95.0, 19.321727383991963), (100.0, 19.643043534825203), (105.0, 20.463909334909605), (110.0, 20.559151107753603), (115.0, 20.522745386934847), (120.0, 13.996284903614873), (125.0, 8.488208767401053), (130.0, 8.321891894516671), (135.0, 8.035019808898115), (140.0, 7.734000809523211), (145.0, 7.462827754728766), (150.0, 7.371285650877788), (155.0, 6.967573388128323)]
data2 = [(25.0, 14.317421395432246), (30.0, 30), (35.0, 22.327769749467343), (40.0, 15.01933317922561), (45.0, 20.3697015026011), (50.0, 20.032839386779997), (55.0, 20.007140368227954), (60.0, 20.232485137811704), (65.0, 20.441385616628132), (70.0, 20.011417965096214), (75.0, 19.95172457569208), (80.0, 20.011418079864477), (85.0, 20.007140368227954), (90.0, 20.414434419756265), (95.0, 20.766804744795348), (100.0, 21.187544722304995), (105.0, 20.045718294663924), (110.0, 16.872030114041998), (115.0, 10.604187474754651), (120.0, 9.802299274713658), (125.0, 9.211932299541955), (130.0, 8.66194900877863), (135.0, 8.310352584462933), (140.0, 8.25502931663987), (145.0, 6.82384406956604), (150.0, 6.501592551828823), (155.0, 6.320182030251865)]
algorithm(data1, True)