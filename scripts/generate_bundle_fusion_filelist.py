#!/usr/bin/env python3

import glob
import os
import sys

# struct FrameStruct {
#    unsigned short messageType;
#    std::vector<unsigned char> colorFrame;
#    std::vector<unsigned char> depthFrame;
#    std::string sceneDesc;
#    unsigned int sensorId;
#    unsigned int deviceId;
#    unsigned int frameId;
#    std::vector<unsigned long> timestamp;
#}

def parse_frame_folder(base_folder):
	# "/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg"
	# "/home/amourao/data/bundle_fusion/apt0/frame-000000.depth.png"
	# frame_id;color_frame_path;depth_frame_path
	# frame_id;color_frame_path;depth_frame_path
	pattern_color = os.path.join(base_folder, "frame-*.color.jpg")
	pattern_depth = os.path.join(base_folder, "frame-*.depth.png")
	frames_color = sorted(glob.glob(pattern_color))
	frames_depth = sorted(glob.glob(pattern_depth))
	if len(frames_color) != len(frames_depth):
		print("Frame count mismatch: {} color; {} depth".format(len(frames_color), len(frames_depth)), file=sys.stderr)
		exit(1)
	print(len(frames_color))
	for i, frames in enumerate(zip(frames_color,frames_depth)):
		print("{};{};{}".format(i, frames[0], frames[1]))


def parse_metadata(base_folder, framerate):
	sceneDesc = base_folder.split("/")[-1]
	
	if not sceneDesc:
		sceneDesc = base_folder.split("/")[-2]

	sensorId = 0
	deviceId = 0
	print("bundle_fusion_{};{};{};{}".format(sceneDesc, sensorId, deviceId, framerate))



def main(argv):
	if len(argv) < 3:
		print("Usage: generate_bundle_fusion_filelist <path> <framerate>", file=sys.stderr)
	path = sys.argv[1]
	framerate = int(sys.argv[2])
	parse_metadata(path, framerate)
	parse_frame_folder(path)



if __name__ == "__main__":
	main(sys.argv)