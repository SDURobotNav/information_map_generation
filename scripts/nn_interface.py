#!/usr/bin/env python3
import pickle
from socket import *
from multiprocessing.connection import Client, Listener
from time import time,sleep
import torch
import numpy as np
# from network import Generator
import network

class map2inform():
    def __init__(self,model_path):
        self.device = "cuda:0"
        self.model = network.Generator(1,1).to(self.device).eval()
        self.model.load_state_dict(torch.load(model_path))

    # get network model output 
    def get_inform(self,map):
        map[map==-1] = 200
        map = torch.from_numpy(map).float().to(self.device)/255
        map = map.unsqueeze(0)
        map = map.unsqueeze(0)
        pred = self.model(map).cpu().detach().numpy()

        return pred[0][0].copy()

    
    