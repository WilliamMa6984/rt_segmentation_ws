# U-Net: Semantic segmentation with PyTorch
# Copyright (C) milesial

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Original author: milesial
# Original source: https://github.com/milesial/Pytorch-UNet

# Modified: 17/05/2026
# Last modified by: William Ma

import os

import numpy as np
import torch
import torch.nn.functional as F
from datetime import datetime

from vision.unet import UNet
from vision.utils.utils import plot_img_and_mask
from vision.utils.data_loading import BasicDataset

from ament_index_python.packages import get_package_share_directory
    
def predict_img(net,
                full_img,
                device,
                scale_factor=1,
                out_threshold=0.5):
    net.eval()
    img = torch.from_numpy(BasicDataset.preprocess(None, full_img, scale_factor, is_mask=False))
    img = img.unsqueeze(0)
    img = img.to(device=device, dtype=torch.float32)

    with torch.no_grad():
        output = net(img)
        output = output.cpu()
        output = F.interpolate(output, (full_img.shape[1], full_img.shape[0]), mode='bilinear')
        if net.n_classes > 1:
            mask = output.argmax(dim=1)
        else:
            mask = torch.sigmoid(output) > out_threshold

    return mask[0].long().squeeze().numpy()

def mask_to_image(mask: np.ndarray, mask_values):
    if isinstance(mask_values[0], list):
        out = np.zeros((mask.shape[-2], mask.shape[-1], len(mask_values[0])), dtype=np.uint8)
    elif mask_values == [0, 1]:
        out = np.zeros((mask.shape[-2], mask.shape[-1]), dtype=bool)
    else:
        out = np.zeros((mask.shape[-2], mask.shape[-1]), dtype=np.uint8)

    if mask.ndim == 3:
        mask = np.argmax(mask, axis=0)

    for i, v in enumerate(mask_values):
        out[mask == i] = v

    return out

def unet_load():
    package_share_directory = get_package_share_directory('vision')
    model_file = os.path.join(package_share_directory, 'checkpoints', 'checkpoint_epoch5.pth')

    net = UNet(n_channels=3, n_classes=2, bilinear=False)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    net.to(device=device)
    state_dict = torch.load(model_file, map_location=device)
    mask_values = state_dict.pop('mask_values', [0, 1])
    net.load_state_dict(state_dict)

    print('Model loaded!')

    return net, mask_values, device