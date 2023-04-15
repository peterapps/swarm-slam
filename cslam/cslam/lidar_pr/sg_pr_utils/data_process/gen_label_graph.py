#!/usr/bin/env python3
# This file is covered by the LICENSE file in the root of this project.

from typing import Dict, Union
import numpy as np
import pcl

learning_map={0 : 0,     # "unlabeled"
    1 : 0,     # "outlier" mapped to "unlabeled" --------------------------mapped       x
    10: 1,     # "car"
    11: 2,     # "bicycle"                                                              x
    13: 5,     # "bus" mapped to "other-vehicle" --------------------------mapped
    15: 3,     # "motorcycle"                                                           x
    16: 5,     # "on-rails" mapped to "other-vehicle" ---------------------mapped
    18: 4,     # "truck"
    20: 5,     # "other-vehicle"
    30: 6,     # "person"                                                               x
    31: 7,     # "bicyclist"                                                            x
    32: 8,     # "motorcyclist"                                                         x
    40: 9,     # "road"
    44: 10,     # "parking"  
    48: 11,    # "sidewalk"
    49: 12,    # "other-ground"
    50: 13,    # "building"
    51: 14,    # "fence"
    52: 0,     # "other-structure" mapped to "unlabeled" ------------------mapped
    60: 9,     # "lane-marking" to "road" ---------------------------------mapped
    70: 15,    # "vegetation"
    71: 16,    # "trunk"
    72: 17,    # "terrain"
    80: 18,    # "pole"
    81: 19,    # "traffic-sign"
    99: 0,     # "other-object" to "unlabeled" ----------------------------mapped
    252: 1,    # "moving-car" to "car" ------------------------------------mapped
    253: 7,    # "moving-bicyclist" to "bicyclist" ------------------------mapped
    254: 6,    # "moving-person" to "person" ------------------------------mapped
    255: 8,    # "moving-motorcyclist" to "motorcyclist" ------------------mapped
    256: 5,    # "moving-on-rails" mapped to "other-vehicle" --------------mapped
    257: 5,    # "moving-bus" mapped to "other-vehicle" -------------------mapped
    258: 4,    # "moving-truck" to "truck" --------------------------------mapped
    259: 5}

max_key = max(learning_map.keys())
remap_lut = np.zeros((max_key + 100), dtype=np.int32)
remap_lut[list(learning_map.keys())] = list(learning_map.values())

node_map={
  1: 0,      # "car"
  4: 1,      # "truck"
  5: 2,      # "other-vehicle"
  11: 3,     # "sidewalk"
  12: 4,     # "other-ground"
  13: 5,     # "building"
  14: 6,     # "fence"
  15: 7,     # "vegetation"
  16: 8,     # "trunk"
  17: 9,     # "terrain"
  18: 10,    # "pole"
  19: 11     # "traffic-sign"
  }

class SemanticGraphGenerator(object):
    def __init__(self):
        """
        ros node spin in init function
        :param pub_rate:
        :param pub_topic:
        """
    
    def gen_labels(self, scan: np.ndarray, label: np.ndarray) -> np.ndarray:

        scan = scan.reshape((-1, 4))
        # put in attribute
        points = scan[:, 0:4]  # get xyzr
        
        label = label.reshape((-1))

        if label.shape[0] == points.shape[0]:
            sem_label = label & 0xFFFF  # semantic label in lower half
            inst_label = label >> 16  # instance id in upper half
            assert ((sem_label + (inst_label << 16) == label).all())
        else:
            print("Points shape: ", points.shape)
            print("Label shape: ", label.shape)
            raise ValueError("Scan and Label don't contain same number of points")

        sem_label = remap_lut[sem_label]
        sem_label_set = list(set(sem_label))
        sem_label_set.sort()

        # Start clustering
        cluster = []
        inst_id = 0
        for id_i, label_i in enumerate(sem_label_set):
            index = np.argwhere(sem_label == label_i)
            index = index.reshape(index.shape[0])
            sem_cluster = points[index, :]

            tmp_inst_label = inst_label[index]
            tmp_inst_set = list(set(tmp_inst_label))
            tmp_inst_set.sort()

            if label_i in [9, 10]:    # road/parking, dont need to cluster
                inst_cluster = sem_cluster
                inst_cluster = np.concatenate((inst_cluster, np.full((inst_cluster.shape[0],1), label_i, dtype=np.uint32)), axis=1)

                inst_cluster = np.concatenate((inst_cluster, np.full((inst_cluster.shape[0],1), inst_id, dtype=np.uint32)), axis=1)
                inst_id = inst_id + 1
                cluster.extend(inst_cluster)  # Nx6                
                continue
                
            elif label_i in [0,2,3,6,7,8]:    # discard
                continue
            
            elif len(tmp_inst_set) > 1 or (len(tmp_inst_set) == 1 and tmp_inst_set[0] != 0):     # have instance labels
                for id_j, label_j in enumerate(tmp_inst_set):
                    points_index = np.argwhere(tmp_inst_label == label_j)
                    points_index = points_index.reshape(points_index.shape[0])

                    if len(points_index) <= 20:
                        continue
                    inst_cluster = sem_cluster[points_index, :]
                    inst_cluster = np.concatenate((inst_cluster, np.full((inst_cluster.shape[0],1), label_i, dtype=np.uint32)), axis=1)

                    inst_cluster = np.concatenate((inst_cluster, np.full((inst_cluster.shape[0],1), inst_id, dtype=np.uint32)), axis=1)
                    inst_id = inst_id + 1
                    cluster.extend(inst_cluster)
            else:    # Euclidean cluster

                if label_i in [1, 4, 5, 14]:     # car truck other-vehicle fence
                    cluster_tolerance = 0.5
                elif label_i in [11, 12, 13, 15, 17]:    # sidewalk other-ground building vegetation terrain
                    cluster_tolerance = 2
                else:
                    cluster_tolerance = 0.2

                if label_i in [16, 19]:    # trunk traffic-sign
                    min_size = 50
                elif label_i == 15:     # vegetation
                    min_size = 200
                elif label_i in [11, 12, 13, 17]:    # sidewalk other-ground building terrain
                    min_size = 300
                else:
                    min_size = 100

                # print(cluster_tolerance, min_size)
                cloud = pcl.PointCloud(sem_cluster[:, 0:3])
                tree = cloud.make_kdtree()
                ec = cloud.make_EuclideanClusterExtraction()
                ec.set_ClusterTolerance(cluster_tolerance)
                ec.set_MinClusterSize(min_size)
                ec.set_MaxClusterSize(50000)
                ec.set_SearchMethod(tree)
                cluster_indices = ec.Extract()

                for j, indices in enumerate(cluster_indices):
                    inst_cluster = np.zeros((len(indices), 4), dtype=np.float32)
                    inst_cluster = sem_cluster[np.array(indices), 0:4]
                    inst_cluster = np.concatenate((inst_cluster, np.full((inst_cluster.shape[0],1), label_i, dtype=np.uint32)), axis=1)

                    inst_cluster = np.concatenate((inst_cluster, np.full((inst_cluster.shape[0],1), inst_id, dtype=np.uint32)), axis=1)
                    inst_id = inst_id + 1
                    cluster.extend(inst_cluster) # Nx6

        cluster = np.array(cluster)
        return cluster      

    def gen_graphs(self, scan: np.ndarray) -> Dict[str, Union[float, int]]: 
        inst = scan[:, -1] # get instance label
        inst_label_set = list(set(inst))  # get nums of inst
        inst_label_set.sort()

        nodes = []  # graph node
        edges = []  # graph edge
        weights = []  # graph edge weights
        cluster = []  # cluster -> node
        centers = []
        for id_i in range(len(inst_label_set)):
            index = np.argwhere(inst_label_set[id_i] == inst) # query cluster by instance label
            index = index.reshape(index.shape[0])
            inst_cluster = scan[index, :]
            sem_label = list(set(inst_cluster[:, -2])) # get semantic label
            assert len(sem_label) == 1 # one instance cluster should have only one semantic label
            if int(sem_label[0]) in node_map.keys():
                cluster.append(inst_cluster[:, :3])
                node_label = node_map[int(sem_label[0])]  # add node
                nodes.append(int(node_label))
                cluster_center = np.mean(inst_cluster[:, :3], axis=0)
                centers.append((cluster_center.tolist()))
            elif int(sem_label[0]) == 9 or int(sem_label[0]) == 10: # ignore "road" and "parking"
                continue
            else:
                print("wrong semantic label: ", sem_label[0])
                exit(-1)

        dist_thresh = 5 # less than thresh, add an edge between nodes

        for i in range(len(cluster)-1):
            for j in range(i+1, len(cluster)):
                pc_i = cluster[i]
                pc_j = cluster[j]
                center_i = np.mean(pc_i, axis=0)
                center_j = np.mean(pc_j, axis=0)
                center = np.mean([center_i, center_j], axis=0)  # centroid of the cluster

                index1 = np.argmin(np.linalg.norm(center - pc_i[:,None], axis=-1), axis=0)
                index2 = np.argmin(np.linalg.norm(center - pc_j[:,None], axis=-1), axis=0)
                min_dis = np.linalg.norm(pc_i[index1] - pc_j[index2], axis=-1)

                if min_dis <= dist_thresh:
                    edges.append([i, j])  # add edge
                    weight = float(1-min_dis/dist_thresh) #  w = 1 - d/d_thresh [0~5m] -> [1~0]
                    weights.append(weight) # add edge_weight
                else:
                    pass

        # generate graph
        graph = {"nodes": nodes,
                 "edges": edges,
                 "weights": weights,
                 "centers": centers
                }
        return graph
