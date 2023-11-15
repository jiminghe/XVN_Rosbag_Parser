#sudo apt update
#sudo apt install ffmpeg
#pip install opencv-python


import os
import cv2
import rosbag
import shutil
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Function to process a single bag file
def process_bag_file(bag_file, topic, output_folder):
    image_dir = os.path.join(output_folder, 'extracted_images')
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)

    bridge = CvBridge()
    concat_file = os.path.join(output_folder, 'input_images.txt')
    with open(concat_file, 'w') as f:
        f.write('ffconcat version 1.0\n')

    prev_stamp = None
    with rosbag.Bag(bag_file, 'r') as bag:
        topics = bag.get_type_and_topic_info()[1].keys()  # Get the list of topics
        if topic not in topics:
            print(f"Topic '{topic}' not found in '{bag_file}'. Skipping file.")
            return

        for topic, msg, t in bag.read_messages(topics=[topic]):
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except Exception as e:
                print(e)
                continue

            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            image_filename = f"{msg.header.stamp.secs}_{msg.header.stamp.nsecs}.png"
            image_path = os.path.join(image_dir, image_filename)
            cv2.imwrite(image_path, rotated_image)

            if prev_stamp is not None:
                duration = (msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9) - \
                           (prev_stamp.secs + prev_stamp.nsecs * 1e-9)
            else:
                duration = 0

            with open(concat_file, 'a') as f:
                if duration > 0:
                    f.write(f"duration {duration}\n")
                f.write(f"file '{image_path}'\n")

            prev_stamp = msg.header.stamp

    # Split the file name from its extension
    base_name_without_ext = os.path.splitext(os.path.basename(bag_file))[0]
    output_video = os.path.join(output_folder, base_name_without_ext + '.mp4')
    os.system(f"ffmpeg -f concat -safe 0 -i {concat_file} -vsync vfr -pix_fmt yuv420p {output_video}")

    shutil.rmtree(image_dir)
    os.remove(concat_file)

# Main function to process all bag files in a folder
def main(input_folder):
    topic = '/camera/lowres/image'
    mp4_export_folder = os.path.join(input_folder, 'mp4_export')
    if not os.path.exists(mp4_export_folder):
        os.makedirs(mp4_export_folder)

    for file in os.listdir(input_folder):
        if file.endswith('.bag'):
            bag_file_path = os.path.join(input_folder, file)
            print(f"Processing {bag_file_path}...")
            process_bag_file(bag_file_path, topic, mp4_export_folder)

if __name__ == '__main__':
    # Use the provided input folder or the current directory if no input is provided
    input_folder = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    main(input_folder)
