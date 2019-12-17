import numpy as np
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import scipy.stats as sts
import pcl

from sklearn.neighbors.kde import KernelDensity
from scipy.spatial import ConvexHull
from sklearn.cluster import KMeans, DBSCAN


def clustering(points_top_layer_3d, obj_length, obj_width, max_number_of_objects):

    if type(points_top_layer_3d) is list:
        points_top_layer_3d = np.asarray(points_top_layer_3d)

    points = points_top_layer_3d[:, [0, 1]]  # process data in xy plane
    points_top_layer_2d = filter_outlier_dbscan(points, 0.06)

    if points_top_layer_3d.shape[0] > 10:   # more than 10 data, enough data
        #cluster_centers, clusters, clusters_rotated_3d = [], [], []
        cluster_estimation = get_best_estimation(points_top_layer_2d, obj_width, obj_length, max_number_of_objects)

    return cluster_estimation


def get_best_estimation(points, obj_width, obj_length, max_number_of_objects):
    """
    :param points: input point from the toplayer (2d)
    :param obj_length: Length of the object
    :param obj_width: Width of the object
    :param max_number_of_objects:
    :return:
    """
    possible_numbers = get_estimated_numbers(points, max_number_of_objects, obj_width * obj_length)
    hull = ConvexHull(points)
    print "I am in the function get_best_estimation"

    prob_list, labels_of_corr_clusters_list, corr_clusters_list = [], [], []

    for k in possible_numbers:
        prob, labels_of_corr_clusters, corr_clusters = get_cluster_estimate(points, obj_length, obj_width, k)
        prob_list.append(prob)
        labels_of_corr_clusters_list.append(labels_of_corr_clusters)
        corr_clusters_list.append(corr_clusters)
        #print "k= ", k, " prob= ", prob
    
    best_prob_idx = np.argmax(np.asarray(prob_list))
    best_cluster_estimation = corr_clusters_list[best_prob_idx]
    best_labels = labels_of_corr_clusters_list[best_prob_idx]
    final_cluster_estimation = best_cluster_estimation[:]
    print "The best estimation is K = ", possible_numbers[best_prob_idx]

    '''
    for c in best_cluster_estimation:
        bounding_box = bounding_rectangle(c)
        count_min = 0
        for p1 in bounding_box:
            min_dist_to_hull = get_min_distance_to_hull(p1, hull)
            if min_dist_to_hull < 0.01:
                count_min = count_min + 1
        if count_min < 3:   # at least 3 bounding box corners should be very close to the hull border
            final_cluster_estimation.remove(c)
  
    return final_cluster_estimation
    '''
    return best_cluster_estimation


def get_min_distance_to_hull(point, hull):
    # convex hull's hyperplanes(a line in 2d) parameters [A, B, C] A*x+ B*y +C = 0
    # projection of a point (x0, y0) on the line = (x, y) = (x0 + A*t, y0 + B*t)
    # A*(x0 + A*t) + B*(y0 + B*t) + C = 0
    # => t=-(C + A*x0 + B*y0)/(A**2+B**2)
    # distance to the hyperplanes is |A*x0 +B*y0+C|/sqrt(A**2+B**2)
    dist_list = []
    for eq in hull.equations:
        dist = abs(eq[-1] + np.dot(eq[:-1], point))/np.sqrt(np.sum(eq[:-1] ** 2))
        #t = -(eq[-1] + np.dot(eq[:-1], point)) / (np.sum(eq[:-1] ** 2))
        #pt_proj = point + eq[:-1] * t
        dist_list.append(dist)
    return min(dist_list)


def get_cluster_estimate(points, object_length, object_width, possible_k):
    """
    :param points: input point cloud after filter out outliers
    :param object_length: Length of a single object
    :param object_width: Width of a single object
    :param possible_k:
    :return: correct_predict : True or False
    """
    # labels for all the input points
    #predicted_labels = KMeans(n_clusters=possible_k, init='k-means++').fit_predict(points)
    kmeans = KMeans(n_clusters=possible_k, init='k-means++').fit(points)
    object_area = object_width * object_length
    labels = set(kmeans.labels_)

    labels_of_corr_clusters, bounding_box_of_corr_clusters = [], []
    corr_clusters_list = []

    for label in labels:
        single_cluster = filter_outlier_dbscan(points[np.where(kmeans.labels_ == label)], 0.08)
        cluster_area = ConvexHull(single_cluster).volume
        #print cluster_area / object_area
        if 0.8 < (cluster_area / object_area) < 1.2:

            box_points = bounding_rectangle(single_cluster)
            edges = box_points[1:] - box_points[:-1]
            dist = [np.sum(np.square(e), axis=0) for e in edges]
            cluster_length = np.sqrt(max(dist))
            cluster_width = np.sqrt(min(dist))


            if 0.8 < cluster_length/object_length < 1.2 and 0.8 < cluster_width/object_width < 1.2:
                labels_of_corr_clusters.append(label)
                bounding_box_of_corr_clusters.append(box_points)
                corr_clusters_list.append(single_cluster)

    number_of_corr_clusters = len(labels_of_corr_clusters)
    prob = float(number_of_corr_clusters)/len(labels)

    if prob <= 0.3:
        prob = 0

    return prob, kmeans.labels_, corr_clusters_list


def get_orientation(box_corners):
    """
    find orientation of the bounding box
    :param points: input point cloud
    :return: phi: rotation around z
    """
    diff = box_corners[1:] - box_corners[:-1]
    dist = np.sqrt(diff[:, 1]**2 + diff[:, 0]**2)
    min_dist_idx = np.argmin(dist)
    phi = np.arctan2(diff[min_dist_idx, 1], diff[min_dist_idx, 0])
    if phi < 0:
        return phi + np.pi
    else:
        return phi


def get_estimated_numbers(points, max_number, object_area):
    """
    :param object_area:
    :param max_number:
    :param points: array (2d)
    :return est_numbers: a list contains possible numbers of objects
    """

    # generate convex hull
    points = filter_outlier_dbscan(points, 0.06)
    object_hull = ConvexHull(points)
   
    est_number, prob= [], []
    # estimate possible number of clusters
   
    for k in range(max_number-1):
        #if object_hull.volume >= (k + 1) * object_area * 0.7: # maybe for water

        if object_hull.volume >= (k + 1) * object_area * 0.85:
            est_number.append(k + 1)
    est_number.reverse()
    return est_number


def bounding_rectangle(points):
    """
    find the bounding box (rectangle) which fits a set of points
    :param points: input point cloud (in xy plane)
    :return:four corners of the bounding box
    """

    pi_half = np.pi / 2.

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = hull_points[1:] - hull_points[:-1]
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi_half))
    angles = np.unique(angles)

    # find rotation matrices
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles - pi_half),
        np.cos(angles + pi_half),
        np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))
    # apply rotations to the convexhull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    box = np.zeros((4, 2))  # coordinators of the four corners
    box[0] = np.dot([x1, y2], r)
    box[1] = np.dot([x2, y2], r)
    box[2] = np.dot([x2, y1], r)
    box[3] = np.dot([x1, y1], r)

    return box


def find_peaks(kde, bins):
    """
    :param kde: the smoothed histogram after applying Kernal Density Estimation(Gaussian)
    :param bins:
    :return: peaks of the smoothed histogram
    """
    max_kde_bins = []

    for idx in range(len(kde) - 1):
        if kde[idx] > 0.1:
            if idx == 0:
                if kde[idx] < kde[idx + 1]:
                    max_kde_bins.append(bins[idx + 1])
                elif kde[idx] > kde[idx + 1]:
                    max_kde_bins.append(bins[idx])
            else:
                if kde[idx] > kde[idx + 1] and kde[idx] > kde[idx - 1]:
                    max_kde_bins.append(bins[idx])

    peaks_bins = [p for p in max_kde_bins if abs(p) > 0.1]
    return max_kde_bins
    #return peaks_bins


def extract_top_layer(data, getObjectHeight = False, aquador= False):
    x_list, y_list, z_list, xyz_list = [], [], [], []
    cloud_list = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    

    # extract x,y,z and store them in separate lists.
    for p in cloud_list:
        x_list.append(p[0])
        y_list.append(p[1])
        z_list.append(p[2])

        # xyz_list.append([p[2], p[1], p[0]])

    # using KDE to get the peaks of histogram of depth data.
    histogram, bins, patches = plt.hist(z_list, color='blue', edgecolor='black', bins=int(100), density=True)
    kde = sts.gaussian_kde(z_list)

    pdf = kde.pdf(bins)
    peaks = find_peaks(pdf, bins)

    #print "peaks", peaks
    plt.plot(bins, pdf)

    # extract all the points whose depth(z) are around "peaks[0]"(first peak), "peaks[-1]"(the floor)
    threshold = 0.02 # threshold is +-m
    floor_points = []
    for i in range(len(z_list)):
        if peaks[-1] - threshold <= z_list[i] <= peaks[-1] + threshold:
            xyz_list.append([x_list[i], y_list[i], z_list[i]])
        if getObjectHeight and ( -threshold <= z_list[i] <=  threshold):
            floor_points.append([x_list[i], y_list[i], z_list[i]])
    
    #p = pcl.PointCloud()
    #p.from_list(xyz_list)
    #p.to_file("/home/jiongrui/catkin_ws/src/camera_data/scripts/python/plane.pcd")
    
    object_inlier_array = filter_outlier_dbscan(xyz_list, 0.07)
    if not aquador:
        if getObjectHeight:
            floor_inlier_array = filter_outlier_dbscan(floor_points, 0.07)
            return object_inlier_array, floor_inlier_array
        else:
            return object_inlier_array
    else:
        if getObjectHeight:
            floor_inlier_array = filter_outlier_dbscan(floor_points, 0.07)
            
            return object_inlier_array, floor_inlier_array, peaks[-1] # return the height of the water bottle
        else:
            return object_inlier_array, peaks[-1]


def filter_outlier_dbscan(pointcloud3d, e, return_outlier=False):
    pcd = np.asarray(pointcloud3d)
    pointcloud = pcd[:, [0, 1]]  # only checking x,y
    outlier, inlier = [], []
    dbscan = DBSCAN(eps=e, min_samples=30).fit(pointcloud)
    labels = dbscan.labels_
    inlier_label = get_inlier_label(labels)

    for i in range(len(labels)):
        if labels[i] != inlier_label:
            #outlier.append(pointcloud[i])
            outlier.append(pointcloud3d[i])
        else:
            #inlier.append(pointcloud[i])
            inlier.append(pointcloud3d[i])

    outlier_array = np.asarray(outlier)
    inlier_array = np.asarray(inlier)

    if return_outlier:
        return inlier_array, outlier_array
    else:
        return inlier_array


def get_inlier_label(labels):
    max_count = 0
    uni_labels = list(set(labels))
    inlier_label = uni_labels[0]
    for l in uni_labels:
        c = list(labels).count(l)
        if c >= max_count:
            max_count = c
            inlier_label = l
    return inlier_label


def get_distance(p1, p2):
    d = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return d
