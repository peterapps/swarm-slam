import torch
import torch.nn as nn

from cslam.lidar_pr.sg_pr_utils.layers_batch import AttentionModule, TenorNetworkModule
import cslam.lidar_pr.sg_pr_utils.dgcnn as dgcnn


class SG(torch.nn.Module):
    """
    SimGNN: A Neural Network Approach to Fast Graph Similarity Computation
    https://arxiv.org/abs/1808.05689
    """

    def __init__(self, args, number_of_labels, device):
        """
        :param args: Arguments object.
        :param number_of_labels: Number of node labels.
        :param device: torch device object
        """
        super(SG, self).__init__()
        self.args = args
        self.number_labels = number_of_labels
        self.device = device
        self.setup_layers()

    def calculate_bottleneck_features(self):
        """
        Deciding the shape of the bottleneck layer.
        """
        self.feature_count = self.args.tensor_neurons

    def setup_layers(self):
        """
        Creating the layers.
        """
        self.calculate_bottleneck_features()
        self.attention = AttentionModule(self.args, self.device)
        self.tensor_network = TenorNetworkModule(self.args, self.device)
        self.fully_connected_first = torch.nn.Linear(self.feature_count, self.args.bottle_neck_neurons).to(device=self.device)
        self.scoring_layer = torch.nn.Linear(self.args.bottle_neck_neurons, 1).to(device=self.device)
        bias_bool = False # TODO
        self.dgcnn_s_conv1 = nn.Sequential(
            nn.Conv2d(3*2, self.args.filters_1, kernel_size=1, bias=bias_bool),
            nn.BatchNorm2d(self.args.filters_1),
            nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)
        self.dgcnn_f_conv1 = nn.Sequential(
            nn.Conv2d(self.number_labels * 2, self.args.filters_1, kernel_size=1, bias=bias_bool),
            nn.BatchNorm2d(self.args.filters_1),
            nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)
        self.dgcnn_s_conv2 = nn.Sequential(
            nn.Conv2d(self.args.filters_1*2, self.args.filters_2, kernel_size=1, bias=bias_bool),
            nn.BatchNorm2d(self.args.filters_2),
            nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)
        self.dgcnn_f_conv2 = nn.Sequential(
            nn.Conv2d(self.args.filters_1 * 2, self.args.filters_2, kernel_size=1, bias=bias_bool),
            nn.BatchNorm2d(self.args.filters_2),
            nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)
        self.dgcnn_s_conv3 = nn.Sequential(
            nn.Conv2d(self.args.filters_2*2, self.args.filters_3, kernel_size=1, bias=bias_bool),
            nn.BatchNorm2d(self.args.filters_3),
            nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)
        self.dgcnn_f_conv3 = nn.Sequential(
            nn.Conv2d(self.args.filters_2 * 2, self.args.filters_3, kernel_size=1, bias=bias_bool),
            nn.BatchNorm2d(self.args.filters_3),
            nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)
        self.dgcnn_conv_end = nn.Sequential(nn.Conv1d(self.args.filters_3 * 2,
                                                      self.args.filters_3, kernel_size=1, bias=bias_bool),
                                            nn.BatchNorm1d(self.args.filters_3), nn.LeakyReLU(negative_slope=0.2)).to(device=self.device)


    def dgcnn_conv_pass(self, x):
        self.k = self.args.K
        xyz = x[:,:3,:] # Bx3xN
        sem = x[:,3:,:]   # BxfxN

        xyz = dgcnn.get_graph_feature(xyz, k=self.k)  #Bx6xNxk
        xyz = self.dgcnn_s_conv1(xyz)
        xyz1 = xyz.max(dim=-1, keepdim=False)[0]
        xyz = dgcnn.get_graph_feature(xyz1, k=self.k)
        xyz = self.dgcnn_s_conv2(xyz)
        xyz2 = xyz.max(dim=-1, keepdim=False)[0]
        xyz = dgcnn.get_graph_feature(xyz2, k=self.k)
        xyz = self.dgcnn_s_conv3(xyz)
        xyz3 = xyz.max(dim=-1, keepdim=False)[0]

        sem = dgcnn.get_graph_feature(sem, k=self.k)  # Bx2fxNxk
        sem = self.dgcnn_f_conv1(sem)
        sem1 = sem.max(dim=-1, keepdim=False)[0]
        sem = dgcnn.get_graph_feature(sem1, k=self.k)
        sem = self.dgcnn_f_conv2(sem)
        sem2 = sem.max(dim=-1, keepdim=False)[0]
        sem = dgcnn.get_graph_feature(sem2, k=self.k)
        sem = self.dgcnn_f_conv3(sem)
        sem3 = sem.max(dim=-1, keepdim=False)[0]

        x = torch.cat((xyz3, sem3), dim=1)

        x = self.dgcnn_conv_end(x)

        x = x.permute(0, 2, 1)  # [node_num, 32]
        return x

    def forward(self, data):
        """
        Forward pass with graphs.
        :param data: Data dictionary.
        :return score: Similarity score.
        """

        features_1 = data["features_1"].to(device=self.device)
        features_2 = data["features_2"].to(device=self.device)

        # features B x (3+label_num) x node_num
        abstract_features_1 = self.dgcnn_conv_pass(features_1) # node_num x feature_size(filters-3)
        abstract_features_2 = self.dgcnn_conv_pass(features_2)  #BXNXF

        pooled_features_1, attention_scores_1 = self.attention(abstract_features_1) # bxfx1
        pooled_features_2, attention_scores_2 = self.attention(abstract_features_2)

        scores = self.tensor_network(pooled_features_1, pooled_features_2)
        scores = scores.permute(0,2,1) # bx1xf
        scores = torch.nn.functional.relu(self.fully_connected_first(scores))

        score = torch.sigmoid(self.scoring_layer(scores)).reshape(-1)

        return score, attention_scores_1, attention_scores_2
