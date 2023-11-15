# dependancy:
# pip install simplekml

import csv
import simplekml
import os
import sys
from datetime import datetime, timezone

# Function to check if the required columns exist in the CSV file
def has_required_columns(csvfile):
    reader = csv.reader(csvfile)
    headers = next(reader, None)
    required_columns = {'Timestamp', 'Latitude', 'Longitude', 'Altitude'}
    return required_columns.issubset(set(headers))

# Function to convert CSV to KML
def convert_csv_to_kml(input_csv, output_kml):
    # Create a KML object
    kml = simplekml.Kml()

    # Define a style for the icon
    icon_style = simplekml.Style()
    icon_style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/pink-blank.png"

    # Define a style for the LineString (path)
    line_style = simplekml.Style()
    line_style.linestyle.width = 3 * 2  # 3 times the default width (default is 2)
    line_style.linestyle.color = simplekml.Color.blue  # Set the color to blue

    # Create a LineString to add a path
    linestring = kml.newlinestring(name="Path")
    linestring.style = line_style  # Apply the custom style to the LineString

    coords = []

    with open(input_csv, 'r') as csvfile:
        if not has_required_columns(csvfile):
            print(f"Skipping {input_csv}: Required columns are missing.")
            return

        csvfile.seek(0)  # Reset file pointer to the beginning after checking columns
        csvreader = csv.DictReader(csvfile)
        previous_timestamp = None
        for row in csvreader:
            timestamp = float(row['Timestamp'])
            if previous_timestamp is None or timestamp - previous_timestamp >= 10:
                lat = float(row['Latitude'])
                lon = float(row['Longitude'])
                alt = float(row['Altitude'])
                dt_object = datetime.fromtimestamp(timestamp, tz=timezone.utc)
                formatted_timestamp = dt_object.strftime("%Y%m%d_%H:%M:%S")
                coords.append((lon, lat, 0))
                pnt = kml.newpoint(name=formatted_timestamp, coords=[(lon, lat, alt)])
                pnt.style = icon_style
                pnt.description = f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}"
                previous_timestamp = timestamp
            else:
                lat = float(row['Latitude'])
                lon = float(row['Longitude'])
                coords.append((lon, lat, 0))

    linestring.coords = coords
    linestring.extrude = 1
    linestring.altitudemode = simplekml.AltitudeMode.clamptoground

    kml.save(output_kml)

# Get the input folder from the command line argument or use the current directory
input_folder = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.realpath(__file__))

# Get a list of all CSV files in the input folder
csv_files = [f for f in os.listdir(input_folder) if os.path.isfile(os.path.join(input_folder, f)) and f.endswith('.csv')]

# Create a subfolder for KML exports if it doesn't exist
kml_export_folder = os.path.join(input_folder, "kml_export")
os.makedirs(kml_export_folder, exist_ok=True)

# Process each CSV file
for csv_file in csv_files:
    input_csv = os.path.join(input_folder, csv_file)
    output_kml = os.path.join(kml_export_folder, os.path.splitext(csv_file)[0] + '.kml')
    convert_csv_to_kml(input_csv, output_kml)
