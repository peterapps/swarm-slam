import yaml
import os

class sgpr_args():
    def __init__(self):

        #arch
        self.keep_node=1
        self.filters_1=64
        self.filters_2=64
        self.filters_3=32
        self.tensor_neurons=16
        self.bottle_neck_neurons=16
        self.K=10

    def load(self,config_file):
        config_args=yaml.safe_load(open(os.path.abspath(config_file)))
        #arch
        self.keep_node=config_args['arch']['keep_node']
        self.filters_1=config_args['arch']['filters_1']
        self.filters_2=config_args['arch']['filters_2']
        self.filters_3=config_args['arch']['filters_3']
        self.tensor_neurons=config_args['arch']['tensor_neurons']
        self.bottle_neck_neurons=config_args['arch']['bottle_neck_neurons']
        self.K=config_args['arch']['K']
