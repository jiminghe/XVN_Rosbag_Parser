#!/bin/bash

# Default working folder to the current directory of the script
WORKING_FOLDER=$(dirname "$(realpath "$0")")

# Function to show usage
usage() {
    echo "Usage: $0 --OUTPUT_FILE <output_bag_file_name> [--WORKING_FOLDER <working_folder>]"
    exit 1
}

# Initialize OUTPUT_FILE variable
OUTPUT_FILE=""

# Parse command line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --OUTPUT_FILE) OUTPUT_FILE="$2"; shift ;;
        --WORKING_FOLDER) WORKING_FOLDER="$2"; shift ;;
        *) usage ;;
    esac
    shift
done


# Set the LOG_FILES_FOLDER based on WORKING_FOLDER
LOG_FILES_FOLDER="$WORKING_FOLDER/log_files"

# Check if .bag files are present in the LOG_FILES_FOLDER
BAG_FILES_FOUND=$(find "$LOG_FILES_FOLDER" -type f -name "*.bag" | wc -l)
if [ "$BAG_FILES_FOUND" -eq 0 ]; then
    echo ".bag files not found in ${LOG_FILES_FOLDER}"
    exit 1
fi


# If OUTPUT_FILE is not provided, use the name of the first .bag file found
if [ -z "$OUTPUT_FILE" ]; then
    # Find the first .bag file
    FIRST_BAG_FILE=$(find "$LOG_FILES_FOLDER" -type f -name "*.bag" | head -n 1)
    if [ -n "$FIRST_BAG_FILE" ]; then
        # Extract the base name without the last 6 characters
        OUTPUT_FILE=$(basename "$FIRST_BAG_FILE" .bag)
        OUTPUT_FILE="${OUTPUT_FILE%_*}"
    else
        echo "Error: No .bag files found to set the output file name."
        exit 1
    fi
fi


# Set the MERGED_BAG_FOLDER based on the OUTPUT_FILE
MERGED_BAG_FOLDER="$LOG_FILES_FOLDER/export_$OUTPUT_FILE"


# Step 1: Merge the .bag files
echo "Working folder = ${LOG_FILES_FOLDER}"
echo "Merging .bag files..."
mkdir -p "$MERGED_BAG_FOLDER"
rosbag-merge --input_paths "$LOG_FILES_FOLDER" --output_path "$MERGED_BAG_FOLDER" --outbag_name "$OUTPUT_FILE" --write_bag
echo "Merging .bag files done."

# Step 2: Export position from bag to a csv
echo "Exporting position from rostopic /fusion_optim/odometry to CSV..."
cd "$WORKING_FOLDER"
python3 exportlla.py "$MERGED_BAG_FOLDER"
echo "Exporting position to csv done."


# Step 3: Convert csv to kml file
echo "Converting CSV to KML..."
python3 csvToKml.py "$MERGED_BAG_FOLDER/csv_export"
echo "Converting CSV to KML done."

# Step 4: Convert '/camera/lowres/image' to mp4 file
echo "Converting '/camera/lowres/image' to MP4..."
python3 rosbag2mp4.py "$MERGED_BAG_FOLDER"
echo "Converting to mp4 done."

echo "Processing done."

