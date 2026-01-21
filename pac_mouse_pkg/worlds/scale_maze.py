import sys
import xml.etree.ElementTree as ET

if len(sys.argv) != 3:
    print("Usage: python3 scale_maze.py input.sdf scale_factor")
    sys.exit(1)

input_file = sys.argv[1]
scale = float(sys.argv[2])

tree = ET.parse(input_file)
root = tree.getroot()

def scale_pose(text):
    values = list(map(float, text.strip().split()))
    values[0] *= scale  # x
    values[1] *= scale  # y
    return " ".join(map(str, values))

def scale_size(text):
    values = list(map(float, text.strip().split()))
    values[0] *= scale  # x
    values[1] *= scale  # y
    return " ".join(map(str, values))

for pose in root.findall(".//pose"):
    pose.text = scale_pose(pose.text)

for size in root.findall(".//size"):
    size.text = scale_size(size.text)

output_file = input_file.replace(".sdf", f"_scaled_{scale}.sdf")
tree.write(output_file)

print(f"Scaled maze written to: {output_file}")
