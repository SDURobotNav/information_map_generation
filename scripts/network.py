#!/usr/bin/env python3
import torch.nn as nn
import torch.nn.functional as F
import torch

# 4.208391298355457e-05
def weights_init_normal(m):
    classname = m.__class__.__name__
    if classname.find("Conv") != -1:
        torch.nn.init.normal_(m.weight.data, 0.0, 0.02)
    elif classname.find("BatchNorm2d") != -1:
        torch.nn.init.normal_(m.weight.data, 1.0, 0.02)
        torch.nn.init.constant_(m.bias.data, 0.0)


class Generator(nn.Module):
    def __init__(self, in_channel, out_channel):
        super(Generator, self).__init__()

        self.cov = nn.Sequential(
            nn.Conv2d(in_channel, 32, 11, stride=1, padding=5),
            nn.BatchNorm2d(32, 0.8),
            nn.ReLU(),
            nn.Conv2d(32, 64, 11, stride=2, padding=5),
            nn.BatchNorm2d(64, 0.8),
            nn.ReLU(),
            nn.Conv2d(64, 128, 11, stride=2, padding=5),
            nn.BatchNorm2d(128, 0.8),
            nn.ReLU()
        )

        self.conv_blocks = nn.Sequential(
            nn.BatchNorm2d(128),
            nn.Upsample(scale_factor=2),
            nn.Conv2d(128, 128, 11, stride=1, padding=5),
            nn.BatchNorm2d(128, 0.8),
            nn.ReLU(),
            nn.Upsample(scale_factor=2),
            nn.Conv2d(128, 64, 11, stride=1, padding=5),
            nn.BatchNorm2d(64, 0.8),
            nn.ReLU(),
            nn.Conv2d(64, out_channel, 11, stride=1, padding=5),
            nn.Sigmoid(),
        )

    def forward(self, z):
        out = self.cov(z)
        out = self.conv_blocks(out)
        return out
