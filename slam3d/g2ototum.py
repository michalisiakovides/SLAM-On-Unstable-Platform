def parse_g2o_file(g2o_file):
    poses = []
    with open(g2o_file, 'r') as file:
        for line in file:
            if line.startswith("VERTEX_SE3:QUAT"):
                parts = line.strip().split()
                id = int(parts[1])
                tx = float(parts[2])
                ty = float(parts[3])
                tz = float(parts[4]) + 0.5
                qx = float(parts[5])
                qy = float(parts[6])
                qz = float(parts[7])
                qw = float(parts[8])
                poses.append((id, tx, ty, tz, qx, qy, qz, qw))
    return poses

def save_tum_file(poses, tum_file):
    with open(tum_file, 'w') as file:
        for pose in poses:
            id, tx, ty, tz, qx, qy, qz, qw = pose
            file.write(f"{id:} {tx:} {ty:} {tz:} {qx:} {qy:} {qz:} {qw:}\n")

def convert_g2o_to_tum(g2o_file, tum_file):
    poses = parse_g2o_file(g2o_file)
    save_tum_file(poses, tum_file)
    print(f"Conversion complete: {g2o_file} -> {tum_file}")

if __name__ == "__main__":
    g2o_file = "pose_graph.g2o"
    tum_file = "tum.txt"
    convert_g2o_to_tum(g2o_file, tum_file)

