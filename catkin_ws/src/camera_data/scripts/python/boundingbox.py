import numpy as np
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import scipy.stats as sts

from scipy.spatial import ConvexHull, Delaunay
from sklearn.neighbors import LocalOutlierFactor
from sklearn.cluster import KMeans, DBSCAN


def clustering(points_toplayer_3d, object_length, object_width, max_nr_clusters):

    if type(points_toplayer_3d) is list:
        points_toplayer_3d = np.asarray(points_toplayer_3d)

    points = points_toplayer_3d[:, [0, 1]]  # process data in xy plane
    points_toplayer_2d = filter_outlier_dbscan(points, 0.06)

    solution = 1
    if points_toplayer_3d.shape[0] > 10:   # more than 10 data, enough data
        cluster_centers, clusters, clusters_rotated_3d = [], [], []

        if solution == 1:
                best_cluster_kmeans, best_cluster_labels, K, best_boxes = best_guess(points_toplayer_2d,
                                                                                     object_width, object_length, max_nr_clusters)
                print "len", len(best_cluster_labels)
                n = 1
                plt.figure(2)
                plt.scatter(points_toplayer_2d[:, 0], points_toplayer_2d[:, 1], s=1)

                for i in best_cluster_labels:
                    cluster_single_3d = points_toplayer_3d[np.where(best_cluster_kmeans == i)]
                    idx = np.where(best_cluster_labels == i)
                    box_points = best_boxes[idx[0][0]]

                    orientation = get_orientation(box_points)
                    cluster_single_2d = cluster_single_3d[:, [0, 1]]
                    cluster_center_data = np.mean(cluster_single_2d, axis=0)
                    cluster_center_box = np.mean(box_points, axis=0)

                    centered_cluster_in = cluster_single_2d - cluster_center_box
                    rotation_matrix = np.array(
                        [[np.cos(orientation), -np.sin(orientation)], [np.sin(orientation), np.cos(orientation)]])
        
                    cluster_single_3d_rotated = np.dot(centered_cluster_in, rotation_matrix)
                    cluster_centers.append(cluster_center_data)
                    clusters_rotated_3d.append(cluster_single_3d_rotated)
                    clusters.append(cluster_single_3d)

                    #plt.show()

                    plt.plot(box_points[:, 0], box_points[:, 1], 'r-')                    #plt.plot(box_points[:, 0], box_points[:, 1], 'r--')
                    #plt.plot(model_box_points[:, 0], model_box_points[:, 1], 'b--')
                    #plt.scatter(model_center[0], model_center[1], marker='x', c="blue")
                    #plt.text(model_center[0] + 0.02, model_center[1] + 0.04, str(format(np.rad2deg(orientation), '.5f')),
                    #         fontsize=9)
                    #plt.text(cluster_center_data[0] - 0.02, cluster_center_data[1] - 0.04, str(format(np.rad2deg(orientation), '.5f')),
                    #         fontsize=9)
                    plt.axis('equal')
                #plt.show()
                n = n+1
                #return cluster_centers, clusters_rotated_3d
                return best_boxes, clusters_rotated_3d

def best_guess(points, width, length, max_nr_clusters):
    possible_nr_clusters = estimated_nr_clusters(points, max_nr_clusters, width * length)
    hull = ConvexHull(points)
    hull_border_points = points[hull.vertices]
    loop = 1
    # find the best guess
    cluster_kmeans, labels, box, probability = [], [], [], []
    for k in possible_nr_clusters:
        one_guess_probs, one_guess_kmeans, one_guess_labels = [], [], []
        one_guess_boxes = []
        for c in range(loop):
            if 0 in one_guess_probs:
                break
            else:
                # kmeans is the predicted label for all the points
                kmeans = KMeans(n_clusters=k, init='k-means++').fit_predict(points)
                one_guess_prob, one_guess_label, one_guess_box = predict_cluster(points,
                                                                                 length, width, kmeans)

                one_guess_probs.append(one_guess_prob)
                one_guess_labels.append(one_guess_label)
                one_guess_kmeans.append(kmeans)
                one_guess_boxes.append(one_guess_box)

        best_cluster_of_one_guess = int(np.argmax(one_guess_probs))

        probability.append(one_guess_probs[best_cluster_of_one_guess])
        cluster_kmeans.append(one_guess_kmeans[best_cluster_of_one_guess])
        labels.append(one_guess_labels[best_cluster_of_one_guess])
        box.append(one_guess_boxes[best_cluster_of_one_guess])
        print 'k=', k, 'prob', one_guess_probs[best_cluster_of_one_guess]

    #best_g = int(np.argmax(probability))-1
    best_index = np.argmax(np.asarray(probability))
    best_predicted_labels = labels[best_index]
    best_predicted_kmeans = cluster_kmeans[best_index]
    best_predicted_nr_cluster = possible_nr_clusters[best_index]
    best_predicted_boxes = box[best_index]

    # check if the 3 box points are close to the convex hull border.
    for box in best_predicted_boxes:
        print len(box)
        for corner_point in box:
            plt.figure(4)
            print "corner point is ", corner_point
            #dist = [distance(corner_point, p) for p in hull_border_points]
            for p in hull_border_points:
                print "distance is ", distance(p, corner_point)
                plt.scatter(points[:,0], points[:,1],s=1)
                plt.plot(corner_point,'x')
                plt.plot(p, 'o')
                plt.axis('equal')
                plt.show()




    return best_predicted_kmeans, best_predicted_labels, best_predicted_nr_cluster, best_predicted_boxes


def estimated_nr_clusters(points, max_nr, object_area):
    '''
    :param points: array
    :param max_nr:
    :return: possilbe number of clusters
    '''

    # generate convex hull
    points = filter_outlier_dbscan(points, 0.06)
    object_hull = ConvexHull(points)
    '''
    plt.figure(2)
    plt.scatter(points[:, 0], points[:, 1], s=1)
    plt.plot(points[object_hull.vertices, 0], points[object_hull.vertices, 1], 'ro')
    plt.axis('equal')
    plt.show()
    '''

    estimated_number_cluster, probability = [], []
    # estimate possible number of clusters
    for k in range(max_nr):
        # if abs(hull.volume - (k+1)*self.object_area) < 0.1:
        if object_hull.volume >= (k + 1) * object_area * 0.85:
        #if 0.9<hull.volume/(object_area*(k+1))<1.1:
            estimated_number_cluster.append(k + 1)
    estimated_number_cluster.reverse()
    return estimated_number_cluster


def predict_cluster(points, object_length, object_width, predicted_label):
    '''
    :param points: input point cloud after filter out outliers
    :param predicted_label: cluster label for each point
    :param obj_area: area of the object we are dealing with (assuming that we know that in advance)
    :return: correct_predict : True or False
    '''

    single_object_area = object_width * object_length
    labels = set(predicted_label)
    compare_area, correct_cluster_label, boxes = [], [], []
    labels_correct_cluster = []

    for l in labels:
        single_cluster = points[np.where(predicted_label == l)]
        single_cluster = filter_outlier_dbscan(single_cluster, 0.08)
        #single_cluster = filter_outlier_lof(single_cluster)
        single_cluster_area = ConvexHull(single_cluster).volume


        if 0.7 < (single_cluster_area / single_object_area) < 1.3:
            box_points = bounding_rectangle(single_cluster)
            edges = box_points[1:] - box_points[:-1]
            dist = [np.sum(np.square(e), axis=0) for e in edges]
            length = np.sqrt(max(dist))
            width = np.sqrt(min(dist))
            #if times > 1:
                #plt.figure(1)
                #plt.scatter(single_cluster[:,0], single_cluster[:,1], s=1)
                #plt.plot(box_points[:,0], box_points[:,1], 'r-')

            if 0.8 < length/object_length< 1.2 and 0.8 < width/object_width < 1.2:
            #if abs(length - object_length) < 0.035 and abs(width - object_width) < 0.035:
                compare_area.append(abs(single_cluster_area - single_object_area))
                labels_correct_cluster.append(l)
                boxes.append(box_points)

    nr_correct_clusters = len(compare_area)
    nr_predicted_clusters = len(labels)
    prob = float(nr_correct_clusters)/nr_predicted_clusters

    if prob <= 0.3:
        prob = 0
    return prob, labels_correct_cluster, boxes


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


def get_orientation(box_corners):
    '''
    find orientation of the bounding box
    :param points: input point cloud
    :return: phi: rotation around z
    '''
    #box_corners = bounding_rectangle(points)
    diff = box_corners[1:] - box_corners[:-1]
    dist = np.sqrt(diff[:, 1]**2 + diff[:, 0]**2)
    min_dist_idx = np.argmin(dist)
    phi = np.arctan2(diff[min_dist_idx, 1], diff[min_dist_idx, 0])
    if phi < 0:
        return phi + np.pi
    else:
        return phi


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


def filter_outlier_lof(pcl, return_outlier=False, return_if_outlier=False):
    lof = LocalOutlierFactor(n_neighbors=23).fit_predict(pcl)
    inlier = pcl[np.where(lof != -1)]
    outlier = pcl[np.where(lof == -1)]

    outlier_array = np.asarray(outlier)
    inlier_array = np.asarray(inlier)

    if return_if_outlier and return_outlier:
        return inlier_array, outlier_array, len(outlier_array) != 0
    elif return_outlier:
        return inlier_array, outlier_array
    else:
        return inlier_array


def alpha_shape(points, alpha, only_outer=True):
    """
    Compute the alpha shape (concave hull) of a set of points.
    :param points: np.array of shape (n,2) points.
    :param alpha: alpha value.
    :param only_outer: boolean value to specify if we keep only the outer border
    or also inner edges.
    :return: set of (i,j) pairs representing edges of the alpha-shape. (i,j) are
    the indices in the points array.
    """
    assert points.shape[0] > 3, "Need at least four points"

    def add_edge(edges, i, j):
        """
        Add a line between the i-th and j-th points,
        if not in the list already
        """
        if (i, j) in edges or (j, i) in edges:
            # already added
            assert (j, i) in edges, "Can't go twice over same directed edge right?"
            if only_outer:
                # if both neighboring triangles are in shape, it's not a boundary edge
                edges.remove((j, i))
            return
        edges.add((i, j))

    tri = Delaunay(points)
    edges = set()
    # Loop over triangles:
    # ia, ib, ic = indices of corner points of the triangle
    for ia, ib, ic in tri.vertices:
        pa = points[ia]
        pb = points[ib]
        pc = points[ic]
        # Computing radius of triangle circumcircle
        # www.mathalino.com/reviewer/derivation-of-formulas/derivation-of-formula-for-radius-of-circumcircle
        a = np.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = np.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = np.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
        s = (a + b + c) / 2.0
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)
        if circum_r < alpha:
            add_edge(edges, ia, ib)
            add_edge(edges, ib, ic)
            add_edge(edges, ic, ia)
    return edges


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


def distance(p1, p2):
    d = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return d





