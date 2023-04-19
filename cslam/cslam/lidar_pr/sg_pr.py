from typing import Dict, Union

import numpy as np

from rclpy import node

from cslam.lidar_pr.sg_pr_utils.data_process.gen_label_graph import SemanticGraphGenerator

class SGPR:
    """
    TODO: (Scan Context descriptor for point clouds)
    TODO: (From: https://github.com/irapkaist/scancontext)
    """
    def __init__(self, params: Dict, node: node.Node):
        self.params = params
        self.node = node

        num_labels = self.params['frontend.sg_pr.graph.number_of_labels']

        self.global_labels = [i for i in range(num_labels)] # 20
        self.global_labels = {val: index for index, val in enumerate(self.global_labels)}
        self.number_of_labels = len(self.global_labels)

        self.node_num = self.params['frontend.sg_pr.graph.node_num']
        
        self.graph_generator = SemanticGraphGenerator()

        self.enable = self.params['frontend.sg_pr.model'].lower(
        ) != 'disable'

    def compute_embedding(self, keyframe: np.ndarray) -> np.ndarray:

        if self.enable:

            scan, label = keyframe[:,:4], keyframe[:, 4:]
            label = label.astype(np.uint16)

            scan_clusters = self.graph_generator.gen_labels(scan, label)
            graph: Dict[str, Union[float, int]] = self.graph_generator.gen_graphs(scan_clusters)

            # transfer to numpy in torch expected format
            embedding= self._process_graph_embedding(graph)

            embedding = embedding.flatten()

            return embedding

        else:
            # Random descriptor if disabled
            # Use this option only for testing
            return np.random.rand(128) # TODO: What should this size be?

    def _process_graph_embedding(self, data: Dict[str, Union[float, int]]) -> np.ndarray:
        """
        Transferring the data to torch and creating a hash table with the indices, features and target.
        :param data: Data dictionary of 
        :return torch_data: Dictionary of Torch Tensors.
        """

        node_num = len(data["nodes"])
        if node_num > self.node_num:
            sampled_index = np.random.choice(node_num, self.node_num, replace=False)
            sampled_index.sort()
            data["nodes"] = np.array(data["nodes"])[sampled_index].tolist()
            data["centers"] = np.array(data["centers"])[sampled_index]

        elif node_num < self.node_num:
            data["nodes"] = np.concatenate(
                (np.array(data["nodes"]), -np.ones(self.node_num - node_num))).tolist()  # padding 0
            data["centers"] = np.concatenate(
                (np.array(data["centers"]), np.zeros((self.node_num - node_num,3))))  # padding 0

        features = np.expand_dims(np.array(
            [np.zeros(self.number_of_labels).tolist() if node == -1 else [
                1.0 if self.global_labels[node] == label_index else 0 for label_index in self.global_labels.values()]
             for node in data["nodes"]]), axis=0)

        # 1xnode_numx3
        batch_xyz = np.expand_dims(data["centers"], axis=0)

        #  Bxnum_nodex(3+num_label) -> Bx(3+num_label)xnum_node
        xyz_feature = np.concatenate((batch_xyz, features), axis=2).transpose(0,2,1)
        np_data = np.squeeze(xyz_feature)
        # torch_data = torch.FloatTensor(np_data)

        return np_data
    