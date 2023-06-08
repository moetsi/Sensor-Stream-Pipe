#! /usr/bin/env python3

from pathlib import Path
import torch
from torch import nn
import blobconverter
import onnx
from onnxsim import simplify
import sys

# Define the model
class DiffImgs(nn.Module):
    def forward(self, img1, img2):
        # We will be inputting UINT16 but interprets as UINT8
        # So we need to adjust to account of the 8 bit shift
        img1DepthFP16 = 256.0 * img1[:,:,:,1::2] + img1[:,:,:,::2]
        img2DepthFP16 = 256.0 * img2[:,:,:,1::2] + img2[:,:,:,::2]

        # Create binary masks for each image
        # A pixel in the mask is 1 if the corresponding pixel in the image is 0, otherwise it's 0
        img1Mask = (img1DepthFP16 == 0)
        img2Mask = (img2DepthFP16 == 0)

        # If a pixel is 0 in either image, set the corresponding pixel in both images to 0
        img1DepthFP16 = img1DepthFP16 * (~img1Mask & ~img2Mask)
        img2DepthFP16 = img2DepthFP16 * (~img1Mask & ~img2Mask)

        # Compute the difference between the two images
        diff = torch.sub(img1DepthFP16, img2DepthFP16)

        # Square the difference
        # square_diff = torch.square(diff)

        # # Compute the square root of the square difference
        # sqrt_diff = torch.sqrt(square_diff)

        # sqrt_diff[sqrt_diff < 1500] = 0

        return diff

# Instantiate the model
model = DiffImgs()

# Create dummy input for the ONNX export
input1 = torch.randn(1, 1, 320, 544 * 2, dtype=torch.float16)
input2 = torch.randn(1, 1, 320, 544 * 2, dtype=torch.float16)

onnx_file = "diff_images.onnx"

# Export the model
torch.onnx.export(model,               # model being run
                  (input1, input2),    # model input (or a tuple for multiple inputs)
                  onnx_file,        # where to save the model (can be a file or file-like object)
                  opset_version=12,    # the ONNX version to export the model to
                  do_constant_folding=True,  # whether to execute constant folding for optimization
                  input_names = ['input1', 'input2'],   # the model's input names
                  output_names = ['output'])

# Simplify the model
onnx_model = onnx.load(onnx_file)
onnx_simplified, check = simplify(onnx_file)
onnx.save(onnx_simplified, "diff_images_simplified.onnx")

# Use blobconverter to convert onnx->IR->blob
blobconverter.from_onnx(
    model="diff_images_simplified.onnx",
    data_type="FP16",
    shaves=4,
    use_cache=False,
    output_dir="../",
    optimizer_params=[],
    compile_params=['-ip U8'],    
)

blobconverter.from_onnx(
    model="diff_images_simplified.onnx",
    data_type="FP16",
    shaves=4,
    use_cache=False,
    output_dir="./",
    optimizer_params=[],
    compile_params=['-ip U8'],    
)