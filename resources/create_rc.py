import yaml
from argparse import ArgumentParser


def parse_arguments():
    parser = ArgumentParser()
    
    parser.add_argument(
        "-ip", 
        "--robot_ip",
        help = "[REQUIRED] Wired IP address of the OT2 Robot",
        required=True
    )

    parser.add_argument(
        "-rc",
        "--robot-config-path",
        default="/root/robot_config.yaml",
        help="The absolute path destination of the robot config file"
    )

    # parser.p
    return parser.parse_args()


def generate_yaml(ip_address, file_destination):
    
    with open(file_destination, "w") as file:
        yaml.dump([{"ip": ip_address, "model":"OT2", "version":5 }], file)

if __name__ == "__main__":
    args = parse_arguments()
    print("create_rc.py arguments")
    print(args)
    generate_yaml(args.robot_ip, args.robot_config_path)
    
## terminal call:
## python3 create_rc.py -ip $robot_ip