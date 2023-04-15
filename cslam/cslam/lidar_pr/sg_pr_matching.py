from collections import OrderedDict
from typing import Tuple, List, Dict
import numpy as np

from rclpy import node

import torch

from os.path import join, isfile

from ament_index_python import get_package_share_directory
from cslam.cslam.lidar_pr.sg_pr_utils.sg_net import SG
from cslam.cslam.lidar_pr.sg_pr_utils.parser_sg import sgpr_args

class SGPRMatching(object):
    """
        Matching of description graphs using SG-PR network
    """

    def __init__(self, params: Dict, node: node.Node):
        """
        Initialization

        Args:
            params (dict): parameters
            node (ROS 2 node handle): node handle
        """
        self.params = params
        self.node = node
        self.device = self.params['torch_device']
        
        # Set of graphs and associated IDs
        self.graphs = torch.tensor()
        self.item_ids = dict()
        self.nb_items = 0

        # SG-PR initialization
        
        # Get full path for model and additional config files
        self.enable = self.params['frontend.sg_pr.model'].lower() is not None
        if self.enable:
            pkg_folder = get_package_share_directory("cslam")
            self.params['frontend.sg_pr.model'] = join(pkg_folder, 
                self.params['frontend.sg_pr.model'])
            self.params['frontend.sg_pr.model_config'] = join(pkg_folder, 
                self.params['frontend.sg_pr.model_config'])

        # Load config for model
        args = sgpr_args()
        args.load(self.params['frontend.sg_pr.model_config'])
        
        # Create model, to have state loaded
        number_of_labels = self.params['frontend.sg_pr.graph.number_of_labels']
        self.model = SG(self.args, number_of_labels)
        
        # Load model state
        model_file = self.params['frontend.sg_pr.model']
        if isfile(model_file):

            print("=> loading SG-PR model '{}'".format(model_file))
            # original saved file with dataparallel
            state_dict = torch.load(model_file, map_location=lambda storage, loc: storage)# 'cuda:'+str(self.args.gpu)) #'cuda:0'
            # create new dict that does not contain 'module'
            new_state_dict = OrderedDict()
            for k, v in state_dict.items():
                name = k[7:] # remove 'module'
                new_state_dict[name] = v
            # vs self.model.load_state_dict(checkpoint['state_dict']), how are they different?
            # load params
            self.model.load_state_dict(new_state_dict)
            self.model = self.model.to(self.device)
            
        else:
            print("Error: SG-PR model path is incorrect")
            self.node.get_logger().error("Error: SG-PR model path is incorrect {}".format(model_file))
            exit()
        

        self.model.eval()


    def add_item(self, descriptor: np.ndarray, item_id: int):
        """Add item to the matching list

        Args:
            descriptor (np.array): descriptor
            item_id: identification info (e.g., int)
        """

        descriptor_torch = torch.FloatTensor(descriptor, device=self.device)
        
        self.graphs = self.graphs.cat(descriptor_torch)

        self.item_ids[self.nb_items] = item_id
        self.nb_items += 1

    def search(self, query: np.ndarray, k: int) -> Tuple[List[int], np.ndarray]:
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int, np.array): best matches
        """
        if self.nb_items < 1:
            return [None], [None]

        graph_shape = self.graphs.shape[0]

        data = dict()
        data["features_1"] = self.graphs
        data["features_2"] = torch.FloatTensor(query, device=self.device).repeat(graph_shape)

        scores, att_weights_1, att_weights_2 = self.model(data)
        
        scores_topk, idx_topk = torch.topk(scores, k)

        scores_topk_numpy = scores_topk.cpu().detach().numpy()
        idx_topk_numpy = idx_topk.cpu().detach().numpy()
        
        return [self.item_ids[n] for n in idx_topk_numpy], scores_topk_numpy

    def search_best(self, query: np.ndarray) -> Tuple[int, np.ndarray]:
        """Search for the nearest neighbor
            Implementation for compatibily only

        Args:
            query (np.array): descriptor to match

        Returns:
            int, np.array: best match
        """
        if self.nb_items < 1:
            return None, None
            
        idxs, sims = self.search(query, 1)

        return idxs[0], sims[0]
