import cslam.lidar_pr.scancontext_utils as sc_utils
import numpy as np
from scipy import spatial

# SG-PR imports
import os
import sys
curr_dir, _ = os.path.split(os.path.realpath(__file__))
SG_PR_DIR = os.path.join(curr_dir, "..", "..", "..", "SG_PR")
sys.path.append(SG_PR_DIR)
from utils import tab_printer, read_json
from sg_net import SGTrainer
from parser_sg import sgpr_args

class SGPRMatching(object):
    """Nearest Neighbor matching of description vectors
    """

    def __init__(self, shape=[20,60], num_candidates=10, threshold=0.15): 
        """ Initialization
            Default configs are the same as in the original paper 

        """
        self.shape = shape
        self.num_candidates = num_candidates
        self.threshold = threshold

        self.scancontexts = np.zeros((1000, self.shape[0], self.shape[1]))
        self.ringkeys = np.zeros((1000, self.shape[0]))
        self.items = dict()
        self.nb_items = 0

        # SG-PR initialization
        args = sgpr_args()
        args.load(os.path.join(SG_PR_DIR, "config", "config.yml"))
        tab_printer(args)
        self.trainer = SGTrainer(args, False)
        self.trainer.model.eval()


    def add_item(self, descriptor, item):
        """Add item to the matching list

        Args:
            descriptor (np.array): descriptor
            item: identification info (e.g., int)
        """
        sc = descriptor.reshape(self.shape)

        if self.nb_items >= len(self.ringkeys):
            self.scancontexts.resize((2 * len(self.scancontexts), self.shape[0], self.shape[1]),
                                 refcheck=False)
            self.ringkeys.resize((2 * len(self.ringkeys), self.shape[0]),
                                 refcheck=False)
                                 
        rk = sc_utils.sc2rk(sc)

        self.scancontexts[self.nb_items] = sc
        self.ringkeys[self.nb_items] = rk
        self.items[self.nb_items] = item

        self.nb_items = self.nb_items + 1

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

        # step 1
        ringkey_history = np.array(self.ringkeys[:self.nb_items])
        ringkey_tree = spatial.KDTree(ringkey_history)

        query_sc = query.reshape(self.shape)
        ringkey_query = sc_utils.sc2rk(query_sc)
        _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

        # step 2        
        # nn_dist = 1.0 # initialize with the largest value of distance
        nn_score = 0.0 # intitialize with the smalllest score
        nn_idx = None
        nn_yawdiff = None
        for ith in range(self.num_candidates):
            candidate_idx = nncandidates_idx[ith]
            candidate_sc = self.scancontexts[candidate_idx]
            # dist, yaw_diff = sc_utils.distance_sc(candidate_sc, query_sc)
            
            # TODO process candidate_sc and query_sc to the correct format required for eval_feature_pair
            pred, att_weights_1, att_weights_2 = self.trainer.eval_feature_pair(candidate_sc, query_sc)
            score = pred[0]
            
            # if(dist < nn_dist):
            #     nn_dist = dist
            #     nn_yawdiff = yaw_diff
            #     nn_idx = candidate_idx
            if (score > nn_score):
                nn_score = score
                nn_idx = candidate_idx

        if nn_idx is None:
            nn_idx = 0 
            nn_yawdiff_deg = 0
            similarity = 0.0
        else:
            # nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
            # similarity = 1 - nn_dist # For now we return only 1 match, but we could return the n best matches
            similarity = nn_score
        return [self.items[nn_idx]], [similarity]

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