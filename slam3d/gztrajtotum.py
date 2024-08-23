def parse_trajectory(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        lines = infile.readlines()

        timestamp_sec = 0
        timestamp_nsec = 0
        pose_name = ""
        position = {"x": 0.0, "y": 0.0, "z": 0.0}
        orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}

        reading_position = False
        reading_orientation = False

        for line in lines:
            line = line.strip()
            if line.startswith('sec:'):
                timestamp_sec = int(line.split(': ')[1])
            elif line.startswith('nsec:'):
                timestamp_nsec = int(line.split(': ')[1])
            elif line.startswith('name:'):
                pose_name = line.split(': ')[1].strip('"')
            elif line.startswith('position {'):
                reading_position = True
            elif line.startswith('orientation {'):
                reading_orientation = True
            elif reading_position and line.startswith('x:'):
                position['x'] = float(line.split(': ')[1])
            elif reading_position and line.startswith('y:'):
                position['y'] = float(line.split(': ')[1])
            elif reading_position and line.startswith('z:'):
                position['z'] = float(line.split(': ')[1])
            elif reading_orientation and line.startswith('x:'):
                orientation['x'] = float(line.split(': ')[1])
            elif reading_orientation and line.startswith('y:'):
                orientation['y'] = float(line.split(': ')[1])
            elif reading_orientation and line.startswith('z:'):
                orientation['z'] = float(line.split(': ')[1])
            elif reading_orientation and line.startswith('w:'):
                orientation['w'] = float(line.split(': ')[1])
            
            if line == '}':
                if reading_position:
                    reading_position = False
                elif reading_orientation:
                    reading_orientation = False
                else:
                    if pose_name == "example":
                        timestamp = timestamp_sec + timestamp_nsec * 1e-9
                        outfile.write(f"{timestamp} {position['x']} {position['y']} {position['z']} "
                                      f"{orientation['x']} {orientation['y']} {orientation['z']} {orientation['w']}\n")


parse_trajectory('gzgz.txt', 'gzgzf.txt')

