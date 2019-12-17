import numpy as np
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import scipy.stats as sts
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull, Delaunay
from sklearn.neighbors import LocalOutlierFactor
from sklearn.cluster import KMeans, DBSCAN


def clustering3d(inlier_array3d, object_length, object_width, max_nr_cluster ):
    inlier_list2d = []
    clustered_inlier= []
    if len(inlier_array3d) > 10:
        for points in inlier_array3d:
            inlier_list2d.append([points[0],points[1]])
        possible_nr_clusters = estimated_nr_clusters(np.asarray(inlier_list2d), max_nr_cluster, object_length*object_width)
        print "possible numbers of objects:", possible_nr_clusters
        best_cluster_kmeans, K = best_guess(np.asarray(inlier_list2d), possible_nr_clusters,
                                                                             object_width, object_length)
        print "best K:", K
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(K):
            cluster_in = inlier_array3d[np.where(best_cluster_kmeans == i)]
            clustered_inlier.append(inlier_array3d[np.where(best_cluster_kmeans == i)])
            #orientation = get_orientation(cluster_in)
            #plt.scatter(cluster_in[:, 0], cluster_in[:, 1], s=1)
        #plt.show()
            #cluster_final = remove_mean(cluster_in)
            np.savetxt('object_data/water'+str(i)+'.dat', cluster_in)
        return clustered_inlier

def estimated_nr_clusters(points, max_nr, object_area):

    # generate convex hull
    points = filter_outlier_dbscan(points, 0.06)
    hull = ConvexHull(points)
    estimated_number_cluster, probability = [], []

    # estimate possible number of clusters
    for k in range(max_nr):
        # if abs(hull.volume - (k+1)*self.object_area) < 0.1:
        if hull.volume >= (k + 1) * object_area * 0.85:
        #if 0.9<hull.volume/(object_area*(k+1))<1.1:
            estimated_number_cluster.append(k + 1)
    estimated_number_cluster.reverse()
    return estimated_number_cluster

def best_guess(points2d, possible_nr_cluster,width, length):
    cluster_kmeans, labels, box, probability = [], [], [], []

    for k in possible_nr_cluster: #[9, 8, 7, 6,...0]
        kmeans = KMeans(n_clusters=k, init='k-means++').fit_predict(points2d)
        one_guess_prob= predict_cluster(points2d, length, width, kmeans)
        probability.append(one_guess_prob)
        cluster_kmeans.append(kmeans)

    best_cluster_of_one_guess = int(np.argmax(probability))
    return cluster_kmeans[best_cluster_of_one_guess], possible_nr_cluster[best_cluster_of_one_guess]


def predict_cluster(points, object_length, object_width, predicted_label):

    single_object_area = object_width * object_length
    labels = set(predicted_label)
    compare_area, correct_cluster_label = [], []
    for l in labels:

        single_cluster = points[np.where(predicted_label == l)]
        single_cluster_area = ConvexHull(single_cluster).volume

        if 0.8 < (single_cluster_area / single_object_area) < 1.2:
            box_points = bounding_rectangle(single_cluster)
            edges = box_points[1:] - box_points[:-1]
            dist = [np.sum(np.square(e), axis=0) for e in edges]
            length = np.sqrt(max(dist))
            width = np.sqrt(min(dist))

            if 0.8 < length/object_length< 1.2 and 0.8 < width/object_width < 1.2:
                compare_area.append(abs(single_cluster_area - single_object_area))

    nr_correct_clusters = len(compare_area)
    nr_predicted_clusters = len(labels)
    prob = float(nr_correct_clusters)/nr_predicted_clusters

    if prob <= 0.3:
        prob = 0

    return prob


def extract_top_layer(points, dimension='2d', cloud=False):
    x_list, y_list, z_list, xyz_list = [], [], [], []
    cloud_list = pc2.read_points(points, skip_nans=False, field_names=("x", "y", "z"))

    # extract x,y,z and store them in separate lists.
    for p in cloud_list:
        z_list.append(p[0])  # depth
        y_list.append(p[1])
        x_list.append(p[2])

        # xyz_list.append([p[2], p[1], p[0]])

    # using KDE to get the peaks of histogram of depth data.
    plt.figure(0)
    histogram, bins, patches = plt.hist(z_list, color='blue', edgecolor='black', bins=int(100), density=True)
    kde = sts.gaussian_kde(z_list)
    pdf = kde.pdf(bins)
    peaks = find_peaks(pdf, bins)

    # extract all the points whose depth(z) are around "peaks[0]"(first peak)
    threshold = 0.03  # threshold is +-1cm
    xy_list, outlier, inlier = [], [], []
    for i in range(len(z_list)):
        if peaks[0] - threshold <= z_list[i] <= peaks[0] + threshold:
            xy_list.append([x_list[i], y_list[i]])
            xyz_list.append([x_list[i], y_list[i], z_list[i]])

    # Using DBScan to eliminate outliers far way from the main point cloud
    if dimension is '2d':
        inlier_array = filter_outlier_dbscan(xy_list, 0.07)
        return inlier_array
    elif dimension is '3d':
        inlier_array = filter_outlier_dbscan(xyz_list, 0.07)
        if cloud:
            inlier_cloud = pc2.create_cloud_xyz32(points.header, inlier_array.tolist())
            return inlier_cloud, inlier_array
        else:
            return inlier_array
    else:
        print "dimesion should be either 2d or 3d"
        quit()


def find_peaks(kde, bins):
        '''
        :param kde: the smoothed histogram after applying Kernal Density Estimation(Gaussian)
        :param bins:
        :return: peaks
        '''
        max_kde_bins = []

        for idx in range(len(kde)-1):
            if kde[idx] > 0.1:
                if idx == 0:
                    if kde[idx] < kde[idx+1]:
                        max_kde_bins.append(bins[idx+1])
                    elif kde[idx] > kde[idx+1]:
                        max_kde_bins.append(bins[idx])
                else:
                    if kde[idx] > kde[idx+1] and kde[idx] > kde[idx-1]:
                        max_kde_bins.append(bins[idx])

        peaks_bins = [p for p in max_kde_bins if abs(p) > 0.1]
        return peaks_bins

def filter_outlier_dbscan(pcl, e, return_outlier=False):
    outlier, inlier =[], []
    dbscan = DBSCAN(eps=e, min_samples=30).fit(pcl)
    labels = dbscan.labels_
    inlier_label = get_inlier_label(labels)
    for i in range(len(labels)):
        if labels[i] != inlier_label:
            outlier.append(pcl[i])
        else:
            inlier.append(pcl[i])

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

def remove_mean(pointList):
    pointArray = np.asarray(pointList)
    pointArray = pointArray - np.mean(pointArray, axis=0)
    return pointArray.tolist()


def bounding_rectangle(points):
    '''
    find bounding box (rectangle) which fits a set of points
    :param points: input point cloud
    :return:four corner points of the bounding box
    '''

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
    # apply rotations to the hull
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

    box = np.zeros((4, 2))
    box[0] = np.dot([x1, y2], r)
    box[1] = np.dot([x2, y2], r)
    box[2] = np.dot([x2, y1], r)
    box[3] = np.dot([x1, y1], r)

    return box

def get_orientation(points):
    '''
    find orientation of the bounding box
    :param points: input point cloud
    :return: phi: rotation around z
    '''
    box_corners = bounding_rectangle(points)
    diff = box_corners[1:] - box_corners[:-1]
    dist = np.sqrt(diff[:, 1]**2 + diff[:, 0]**2)
    min_dist_idx = np.argmin(dist)
    phi = np.arctan2(diff[min_dist_idx, 1], diff[min_dist_idx, 0])
    if phi < 0:
        return phi + np.pi
    else:
        return phi