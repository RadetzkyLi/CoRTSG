import os
import argparse
import carla

VISIBLE_TOWNS = [
    "Town01",
    "Town02",
    "Town03",
    "Town04",
    "Town05",
    "Town06",
    "Town07",
    "Town10HD"
]

def export_carla_map(town_name, save_dir):
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    if town_name is not None:
        try:
            # Load the desired CARLA map
            world = client.load_world(town_name)
        except RuntimeError:
            print(f" %s is not found in your CARLA repo!"%(town_name))
    else:
        world = client.get_world()

    carla_map = world.get_map()

    save_path = os.path.join(save_dir, town_name+".xodr")
    carla_map.save_to_disk(save_path)
    print("CARLA map exported to {0} in OpenDRIVE format.".format(save_path))

    # # Convert the map to OpenDRIVE
    # opendrive_data = carla_map.to_opendrive()

    # # Save the OpenDRIVE data to a file
    # file_name = 'carla_map.xodr'
    # with open(file_name, 'w') as file:
    #     file.write(opendrive_data)

    # print(f"CARLA map exported to {file_name} in OpenDRIVE format.")


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='CARLA map converter')
    argparser.add_argument(
        '--town',
        help = 'town to be converted'
        )
    argparser.add_argument(
        '--save_dir',
        help="dir to save the converted map"
        )
    args = argparser.parse_args()
    export_carla_map(args.town, args.save_dir)