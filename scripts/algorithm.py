import numpy as np
import random
import math
import matplotlib.pyplot as plt

initial_guess = [np.array([-8, 5]), np.array([8, 5]), np.array([-8, 10]), np.array([8, 10]),np.array([0, 20]) ]
TOL1 = 1.2
TOL2 = 1

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

def plot_data(mu, clusters):
    for arr in mu:
        plt.scatter(arr[0], arr[1], c='black', s=125)

    colors = ['green', 'blue', 'red', 'orange', 'violet']
    i = 0
    for key, value in clusters.items():
        for arr in value:
            plt.scatter(arr[0], arr[1], c=colors[i], s=75)
        i += 1

    plt.grid(True)
    axes = plt.gca()
    axes.set_xlim([-20,20])
    axes.set_ylim([-3,30])

    plt.show()

def stage_1(mu, clusters):
    
    #Create numpy array
    mu_x = []
    mu_y = []
    for arr in mu:
        mu_x.append(arr[0])
        mu_y.append(arr[1])
    mu_arr = np.array([mu_x, mu_y])

    #Split into sides
    mu_left = mu_arr[:,mu_arr[0,:]<-TOL1]
    mu_right = mu_arr[:,mu_arr[0,:]>TOL1]
    mu_middle = mu_arr[:,np.all([mu_arr[0,:]<TOL1,mu_arr[0,:]>-TOL1], axis=0)]

    #Check Dimensions
    if not np.shape(mu_left)[1]==2 or not np.shape(mu_right)[1]==2 or not np.shape(mu_middle)[1]==1:
        print 'Error Occurred in Stage 1 Algorithm'
        print {'mu_left': mu_left, 'mu_right': mu_right, 'mu_middle': mu_middle}
        return False

    #Check left side
    mu_left[np.argsort(mu_left[1, :])]
    if abs(mu_left[0, 1] - mu_left[0, 0]) > TOL2:
        junction_left = True
    else:
        junction_left = False

    #Check right side
    mu_right[np.argsort(mu_right[1, :])]
    if abs(mu_right[0, 1] - mu_right[0, 0]) > TOL2:
        junction_right = True
    else:
        junction_right = False

    return ({'mu_left': mu_left, 'mu_right': mu_right, 'mu_middle': mu_middle}, [junction_left, junction_right])

def algorithm(data, plot_bool):
    x0 = data[0][1]*math.cos(deg_to_rad(data[0][0]))
    y0 = data[0][1]*math.sin(deg_to_rad(data[0][0]))
    X = np.array([[x0, y0]])
    for i in range(1, len(data)):
        x = data[i][1]*math.cos(deg_to_rad(data[i][0]))
        y = data[i][1]*math.sin(deg_to_rad(data[i][0]))
        X = np.concatenate((X, np.array([[x, y]])), axis=0)
    K = 5

    mu, clusters = find_centers(X,K)

    result = stage_1(mu, clusters)
    if result:
        print 'Mu', result[0]
        print 'Junction', result[1]

    if plot_bool:
        plot_data(mu, clusters)

    return result[1]