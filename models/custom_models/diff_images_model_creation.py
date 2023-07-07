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
        # For each image, the depth data seems to be split across every other channel.
        # These two lines combine the interleaved 8-bit values into a 16-bit floating-point depth image.
        img1DepthFP16 = 256.0 * img1[:,:,:,1::2] + img1[:,:,:,::2]
        img2DepthFP16 = 256.0 * img2[:,:,:,1::2] + img2[:,:,:,::2]

        # These lines create masks where the depth data is zero for both images.
        # Zero depth often indicates an invalid or masked depth value.
        img1Mask = (img1DepthFP16 == 0)
        img2Mask = (img2DepthFP16 == 0)

        # Apply the masks to the depth images.
        # The ~ operator inverts the masks, so we're effectively setting all pixels to zero
        # where the depth was zero in either image.
        img1DepthFP16 = img1DepthFP16 * (~img1Mask & ~img2Mask)
        img2DepthFP16 = img2DepthFP16 * (~img1Mask & ~img2Mask)

        # Calculate the difference between the depth images.
        # Pixels where the depth was zero in either image will also be zero in the difference image.
        diff = torch.sub(img1DepthFP16, img2DepthFP16)

        # Return the difference image.
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