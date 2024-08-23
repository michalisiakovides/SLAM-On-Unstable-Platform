import yaml

def parse_trajectory(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        data = yaml.safe_load_all(infile)
        
        for entry in data:
            # Extracting the timestamp
            timestamp_sec = entry['header']['stamp']['sec']
            timestamp_nanosec = entry['header']['stamp']['nanosec']
            timestamp = timestamp_sec + timestamp_nanosec * 1e-9
            
            # Extracting pose information
            pos_x = entry['pose']['pose']['position']['x']
            pos_y = entry['pose']['pose']['position']['y']
            pos_z = entry['pose']['pose']['position']['z'] + 0.5
            
            ori_x = entry['pose']['pose']['orientation']['x']
            ori_y = entry['pose']['pose']['orientation']['y']
            ori_z = entry['pose']['pose']['orientation']['z']
            ori_w = entry['pose']['pose']['orientation']['w']
            
            # Formatting the line to be written
            line = f"{timestamp} {pos_x} {pos_y} {pos_z} {ori_x} {ori_y} {ori_z} {ori_w}\n"
            outfile.write(line)


parse_trajectory('liolio.txt', 'lioliof.txt')

