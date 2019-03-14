import numpy as np

def segment_points(filtered, puddle_thresh):
    num_points = filtered.shape[0]
    dist_to_all_others = np.ones((num_points, num_points))*np.inf # distance to self stored as infinity
    for i in range(num_points):
        for j in range(num_points):
            if i != j:
                dist_to_all_others[i,j] = np.linalg.norm(filtered[i] - filtered[j])

    clusters = []
    queue = []
    unprocessed_points = range(num_points)
    while len(unprocessed_points)>0:
        print("unprocessed list: " + str(unprocessed_points))
        i = unprocessed_points.pop()
        print("currently evaluating: " + str(i))
        queue.append(i)
        for q in queue:
            print("queue: " + str(queue))
            print("processing: " + str(q))
            dists_to_q = dist_to_all_others[q,:]
            indicies_within_thresh = np.where(dists_to_q < puddle_thresh)[0].tolist()
            print("indicies_within_thresh: " + str(indicies_within_thresh))
            indicies_unqueued = [x for x in indicies_within_thresh if x not in queue]
            print("indicies_unqueued: " + str(indicies_unqueued))
            queue.extend(indicies_unqueued)
        clusters.append(np.array(queue))
        unprocessed_points = [j for j in unprocessed_points if j not in queue] # remove all points that were just added to the cluster
        queue = []
    return clusters

if __name__ == '__main__':
    cloud_1_points = np.random.randn(10,2)*1.5
    cloud_2_points = np.random.randn(10,2)*1.5 + 30
    print("cloud 1 \n" + str(cloud_1_points))
    print("cloud 2 \n" + str(cloud_2_points))
    total_cloud = np.vstack((cloud_1_points, cloud_2_points))
    clusters = segment_points(total_cloud, 5)
    for i in range(len(clusters)):
        print("cluster " + str(i) + "\n" + str(clusters[i]))