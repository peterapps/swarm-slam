#!/usr/bin/env python3
# Loop Closure Detection service
# Abstraction to support multiple implementations of loop closure detection for benchmarking

import torch
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image

from cslam.global_descriptor_loop_closure_detection import GlobalDescriptorLoopClosureDetection


class LoopClosureDetection(Node):
    """ Global image descriptor matching for loop closure detection """

    def __init__(self):
        """Initialization and parameter parsing"""
        super().__init__('loop_closure_detection')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_id', None), 
                        ('max_nb_robots', None),
                        # Neighbor Management
                        ('neighbor_management.max_heartbeat_delay_sec', 5),
                        ('neighbor_management.init_delay_sec', 5),
                        ('neighbor_management.heartbeat_period_sec', 0.5),
                        ('neighbor_management.enable_neighbor_monitoring', False),
                        # Loop Closure Networking Settings
                        ('frontend.enable_intra_robot_loop_closures', False),
                        ('frontend.intra_loop_min_inbetween_keyframes', 10),

                        ('frontend.inter_robot_loop_closure_budget', 5),
                        ('frontend.inter_robot_detection_period_sec', 5),
                        ('frontend.inter_robot_matches_topic', "inter_robot_matches"),
                        ('frontend.use_vertex_cover_selection', True),
                        
                        ('frontend.detection_publication_period_sec', 1.0),
                        ('frontend.detection_publication_max_elems_per_msg', 10), 
                        # Loop Closure Technique Specifics
                        ('frontend.sensor_type', "stereo"),
                        ('frontend.global_descriptor_technique', None),
                        ('frontend.global_descriptors_topic', "global_descriptors"),
                        ('frontend.enable_sparsification', True),
                        ('frontend.similarity_threshold', None),
                        ('frontend.nb_best_matches', 10), 

                        # Visual (NetVLAD & COSPLACE)
                        ('frontend.nn_checkpoint', 'disable'),
                        ('frontend.image_crop_size', None),
                        # NetVLAD
                        ('frontend.netvlad.pca_checkpoint', None), 
                        # Cosplace
                        ('frontend.cosplace.descriptor_dim', 64),
                        ('frontend.cosplace.backbone', "resnet18"),
                        # SG-PR
                        ('frontend.sg_pr.model', None),
                        ('frontend.sg_pr.model_config', None),
                        ('frontend.sg_pr.graph.node_num', None),
                        ('frontend.sg_pr.graph.number_of_labels', None),
                        
                        
                        # Evaluation
                        ('evaluation.enable_logs', False),
                        ('evaluation.enable_sparsification_comparison', False),
                        ])
        self.params = {}

        self.params['robot_id'] = self.get_parameter('robot_id').value
        self.params['max_nb_robots'] = self.get_parameter('max_nb_robots').value
        # Neighbor Management
        self.params["neighbor_management.max_heartbeat_delay_sec"] = self.get_parameter(
            'neighbor_management.max_heartbeat_delay_sec').value
        self.params["neighbor_management.init_delay_sec"] = self.get_parameter(
            'neighbor_management.init_delay_sec').value
        self.params["neighbor_management.heartbeat_period_sec"] = self.get_parameter(
            'neighbor_management.heartbeat_period_sec').value
        self.params["neighbor_management.enable_neighbor_monitoring"] = self.get_parameter(
            'neighbor_management.enable_neighbor_monitoring').value
        # Loop Closure Networking Settings
        self.params['frontend.enable_intra_robot_loop_closures'] = self.get_parameter(
            'frontend.enable_intra_robot_loop_closures').value
        self.params['frontend.intra_loop_min_inbetween_keyframes'] = self.get_parameter(
            'frontend.intra_loop_min_inbetween_keyframes').value
        
        self.params['frontend.inter_robot_loop_closure_budget'] = self.get_parameter(
            'frontend.inter_robot_loop_closure_budget').value
        self.params['frontend.inter_robot_detection_period_sec'] = self.get_parameter(
            'frontend.inter_robot_detection_period_sec').value
        # TODO: Where is inter_robot_matches_topic
        self.params["frontend.use_vertex_cover_selection"] = self.get_parameter(
            'frontend.use_vertex_cover_selection').value

        self.params[
            "frontend.detection_publication_period_sec"] = self.get_parameter(
                'frontend.detection_publication_period_sec').value
        self.params[
            "frontend.detection_publication_max_elems_per_msg"] = self.get_parameter(
                'frontend.detection_publication_max_elems_per_msg').value
        # Loop Closure Technique Specifics
        self.params["frontend.sensor_type"] = self.get_parameter(
            'frontend.sensor_type').value.lower()
        self.params['frontend.global_descriptor_technique'] = self.get_parameter(
            'frontend.global_descriptor_technique').value
        # TODO: Where is global_descriptors_topic
        self.params["frontend.enable_sparsification"] = self.get_parameter(
            'frontend.enable_sparsification').value
        self.params['frontend.similarity_threshold'] = self.get_parameter(
            'frontend.similarity_threshold').value
        self.params['frontend.nb_best_matches'] = self.get_parameter(
            'frontend.nb_best_matches').value
        
        # Visual (NetVLAD & COSPLACE)
        self.params['frontend.nn_checkpoint'] = self.get_parameter(
            'frontend.nn_checkpoint').value
        self.params["frontend.image_crop_size"] = self.get_parameter(
            'frontend.image_crop_size').value
        # NetVLAD
        # TODO: Where is netvlad.pca_checkpoint
        # Cosplace
        self.params["frontend.cosplace.descriptor_dim"] = self.get_parameter(
            'frontend.cosplace.descriptor_dim').value
        self.params["frontend.cosplace.backbone"] = self.get_parameter(
            'frontend.cosplace.backbone').value
        # SG-PR
        self.params['frontend.sg_pr.model'] = self.get_parameter(
            'frontend.sg_pr.model').value
        self.params['frontend.sg_pr.model_config'] = self.get_parameter(
            'frontend.sg_pr.model_config').value
        self.params['frontend.sg_pr.graph.node_num'] = self.get_parameter(
            'frontend.sg_pr.graph.node_num').value
        self.params['frontend.sg_pr.graph.number_of_labels'] = self.get_parameter(
            'frontend.sg_pr.graph.number_of_labels').value
        # Evaluation
        self.params["evaluation.enable_logs"] = self.get_parameter(
            'evaluation.enable_logs').value
        self.params["evaluation.enable_sparsification_comparison"] = self.get_parameter(
            'evaluation.enable_sparsification_comparison').value

        
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        self.params['torch_device'] = self.device

        self.glcd = GlobalDescriptorLoopClosureDetection(
            self.params, self)
        self.inter_robot_detection_timer = self.create_timer(
            self.params['frontend.inter_robot_detection_period_sec'],
            self.glcd.detect_inter, clock=Clock())


if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
