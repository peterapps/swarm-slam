from collections import OrderedDict
import numpy as np
from scipy import spatial

import torch

from os.path import join, exists, isfile, realpath, dirname

from ament_index_python import get_package_share_directory
from cslam.cslam.lidar_pr.sg_pr_utils.sg_net import SG
from cslam.cslam.sg_pr_utils.parser_sg import sgpr_args

# SG-PR imports
# import os
# import sys
# curr_dir, _ = os.path.split(os.path.realpath(__file__))
# SG_PR_DIR = os.path.join(curr_dir, "..", "..", "..", "SG_PR")
# sys.path.append(SG_PR_DIR)
# from utils import tab_printer, read_json
# from sg_net import SGTrainer


class SGPRMatching(object):
    """
    TODO: Nearest Neighbor matching of description vectors
    """

    def __init__(self, params, node): # shape=[20,60], num_candidates=10, threshold=0.15): 
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
        # tab_printer(args)
        
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
            # self.model.cuda(self.args.gpu)

            # self.model = torch.nn.DataParallel(self.model, device_ids=[self.args.gpu])
        else:
            print("Error: SG-PR model path is incorrect")
            self.node.get_logger().error("Error: SG-PR model path is incorrect {}".format(model_file))
            exit()
        

        self.model.eval()


    def add_item(self, descriptor, item_id):
        """Add item to the matching list

        Args:
            descriptor (np.array): descriptor
            item_id: identification info (e.g., int)
        """

        descriptor_torch = torch.FloatTensor(descriptor)
        
        self.graphs = self.graphs.cat(descriptor_torch)

        self.item_ids[self.nb_items] = item_id
        self.nb_items += 1

        # sc = descriptor.reshape(self.shape)

        # if self.nb_items >= len(self.ringkeys):
        #     self.scancontexts.resize((2 * len(self.scancontexts), self.shape[0], self.shape[1]),
        #                          refcheck=False)
        #     self.ringkeys.resize((2 * len(self.ringkeys), self.shape[0]),
        #                          refcheck=False)
                                 
        # rk = sc_utils.sc2rk(sc)

        # self.scancontexts[self.nb_items] = sc
        # self.ringkeys[self.nb_items] = rk
        # self.items[self.nb_items] = item

        # self.nb_items = self.nb_items + 1

    def search(self, query, k):
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int, np.array): best matches
        """
        if self.nb_items < 1:
            return [None], [None]

        query_torch = torch.FloatTensor(query)

        graph_shape = self.graphs.shape[0]
        scores, att_weights_1, att_weights_2 = self.trainer.eval_feature_batch(self.graphs, query_torch.repeat(graph_shape))
        
        scores_topk, idx_topk = torch.topk(scores, k)

        scores_topk_numpy = scores_topk.cpu().detach().numpy()
        idx_topk_numpy = idx_topk.cpu().detach().numpy()
        
        return [self.item_ids[n] for n in idx_topk_numpy], scores_topk_numpy

        # nn_score = 0.0 # intitialize with the smalllest score
        # nn_idx = None
        # for idx, graph in enumerate(self.graphs):
            
        #     score, att_weights_1, att_weights_2 = self.trainer.eval_feature_pair(graph, query_torch)

        #     if (score > nn_score):
        #         nn_score = score
        #         nn_idx = idx

        # if nn_idx is None:
        #     nn_idx = 0 
        #     nn_yawdiff_deg = 0
        #     similarity = 0.0
        # else:
        #     # nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
        #     # similarity = 1 - nn_dist # For now we return only 1 match, but we could return the n best matches
        #     similarity = nn_score
        # return [self.items[nn_idx]], [similarity]


        # # step 1
        # ringkey_history = np.array(self.ringkeys[:self.nb_items])
        # ringkey_tree = spatial.KDTree(ringkey_history)

        # query_sc = query.reshape(self.shape)
        # ringkey_query = sc_utils.sc2rk(query_sc)
        # _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

        # # step 2        
        # # nn_dist = 1.0 # initialize with the largest value of distance
        # nn_score = 0.0 # intitialize with the smalllest score
        # nn_idx = None
        # nn_yawdiff = None
        # for ith in range(self.num_candidates):
        #     candidate_idx = nncandidates_idx[ith]
        #     candidate_sc = self.scancontexts[candidate_idx]
        #     # dist, yaw_diff = sc_utils.distance_sc(candidate_sc, query_sc)
            
        #     # TODO process candidate_sc and query_sc to the correct format required for eval_feature_pair
        #     pred, att_weights_1, att_weights_2 = self.trainer.eval_feature_pair(candidate_sc, query_sc)
        #     score = pred[0]
            
        #     # if(dist < nn_dist):
        #     #     nn_dist = dist
        #     #     nn_yawdiff = yaw_diff
        #     #     nn_idx = candidate_idx
        #     if (score > nn_score):
        #         nn_score = score
        #         nn_idx = candidate_idx

        # if nn_idx is None:
        #     nn_idx = 0 
        #     nn_yawdiff_deg = 0
        #     similarity = 0.0
        # else:
        #     # nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
        #     # similarity = 1 - nn_dist # For now we return only 1 match, but we could return the n best matches
        #     similarity = nn_score
        # return [self.items[nn_idx]], [similarity]

    def search_best(self, query):
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
