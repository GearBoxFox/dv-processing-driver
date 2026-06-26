import xml.etree.ElementTree as ET

def convert_calibration(input_file, json_dict, camera_name):
    # Parse the input XML file
    tree = ET.parse(input_file)
    root = tree.getroot()

    # Extract camera matrix values from the input XML
    print(root.findall("./" + camera_name + "/camera_matrix"))

    for child in root.findall("./" + camera_name + "/distortion_coefficients/"):
        if child.tag == "data":
            data_values = list(child.text.split())
            json_dict["cameras"]["C0"]["distortionCoefficients"] = data_values

def open_json(json_file):
    import json
    with open(json_file, 'r') as f:
        return json.load(f)
    
def save_json(json_dict, json_file):
    import json
    with open(json_file, 'w') as f:
        json.dump(json_dict, f, indent=4)
    

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert calibration XML to JSON format.")
    parser.add_argument("input_file", help="Path to the input XML file.")
    parser.add_argument("json_file", help="Path to the output JSON file.")
    parser.add_argument("camera_name", help="Name of the camera.")

    args = parser.parse_args()

    # print(args.input_file)
    # print(args.json_file)
    # print(args.camera_name)

    # Load the JSON dictionary
    json_dict = open_json(args.json_file)
    # print(json_dict)

    # Convert the calibration data
    convert_calibration(args.input_file, json_dict, args.camera_name)
    print(json_dict)
    save_json(json_dict, args.json_file)