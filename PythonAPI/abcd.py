
from utils.data_storage import DataStorageWriter
from utils.carla_utils import *

import random
import time
# import cv2

from tqdm import tqdm

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


# Use Case Definition

USE_CASE_1 = 1
USE_CASE_2 = 2
USE_CASE_3 = 3
USE_CASE_4 = 4
USE_CASE_5 = 5


# vehicles list
VEHICLES_LIST = ['vehicle.audi.a2',
                 'vehicle.audi.tt',
                 'vehicle.carlamotors.carlacola',
                 'vehicle.citroen.c3',
                 'vehicle.dodge_charger.police',
                 'vehicle.jeep.wrangler_rubicon',
                 #'vehicle.nissan.patrol',
                 'vehicle.ford.mustang',
                 #'vehicle.bmw.isetta',
                 'vehicle.audi.etron',
                 'vehicle.mercedes-benz.coupe',
                 'vehicle.bmw.grandtourer',
                 'vehicle.toyota.prius',
                 #'vehicle.diamondback.century',
                 'vehicle.tesla.model3',
                 'vehicle.seat.leon',
                 'vehicle.lincoln.mkz2017',
                 'vehicle.volkswagen.t2',
                 'vehicle.nissan.micra',
                 'vehicle.chevrolet.impala'
                ]


# Definition of Sensors (Position, Settings)

camera1_parameters = {'x': 0.15, 'y': -0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': -45,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera1', 'sensor_type': 'camera'}

camera2_parameters = {'x': 0.15, 'y': 0.00, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 0,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera2', 'sensor_type': 'camera'}

camera3_parameters = {'x': 0.15, 'y': 0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 45,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera3', 'sensor_type': 'camera'}

camera4_parameters = {'x': -0.2, 'y': 0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 90,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera4', 'sensor_type': 'camera'}

camera5_parameters = {'x': -0.6, 'y': 0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 135,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera5', 'sensor_type': 'camera'}

camera6_parameters = {'x': -0.6, 'y': 0.00, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 180,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera6', 'sensor_type': 'camera'}

camera7_parameters = {'x': -0.6, 'y': -0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 225,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera7', 'sensor_type': 'camera'}

camera8_parameters = {'x': -0.2, 'y': -0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 270,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera8', 'sensor_type': 'camera'}

# birds-eye view
camera10_parameters = {'x': 0, 'y': 0, 'z': 100, 'roll': 0, 'pitch': -90, 'yaw': 90,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera10', 'sensor_type': 'camera'}

# 3rd person view
camera9_parameters = {'x': -7, 'y': 0, 'z': 4, 'roll': 0, 'pitch': -25, 'yaw': 0,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera9', 'sensor_type': 'camera'}

lidar1_parameters = {'x': 0, 'y': 0, 'z': 2.0, 'roll': 0, 'pitch': 0, 'yaw': 0,
                     'channels': 32, 'range': 100.0*200, 'lower_fov': -30, 'upper_fov': 15, 'points_per_second': 56000*10, 'rotation_frequency': 30,
                     'sensor_label': 'lidar', 'sensor_type': 'lidar'}

gnss_parameters = {'x': 1.5, 'y': 0, 'z': 2.4, 'roll': 0, 'pitch': 0, 'yaw': 0,
                   'sensor_label': 'gnss', 'sensor_type': 'gnss'}

# imu_parameters = {'x': 1.5, 'y': 0, 'z': 2.4, 'roll': 0, 'pitch': 0, 'yaw': 0, 'sensor_label': 'imu', 'sensor_type': 'imu'}
# collision_parameters = {'sensor_label': 'collision', 'sensor_type': 'collision'}

# List with all sensors and its configurations to be used in the simulation
SENSOR_LIST = [camera1_parameters, camera2_parameters, camera3_parameters, camera4_parameters, camera5_parameters, camera6_parameters, camera7_parameters, camera8_parameters,
               gnss_parameters,
               camera9_parameters,
               camera10_parameters,
               lidar1_parameters
               ]


def run_use_case(use_case: str, output_file_path: str, sensor_list: list, sim_params: dict,
                 save_rgb_as_jpeg: bool = True) -> None:
    """
        run the simulation for a specific use case


        parameters
        ==========

        use_case: use case number (1-4)

        output_file_path: str

        sensor_list: list with all sensors to be used

        sim_params: dict
            - fps: acquisition frame rate
            - ego_velocity: float m/s 
            - opponents_velocities: list [op1, op2, op3, op4, op5] float m/s must, must be absolute values
    """
    print("> Connecting to localhost:2000")
    client = carla.Client('localhost', 2000)
    client.set_timeout(4.0)

    world = client.get_world()

    print("> Configuring weather conditions")
    weather = world.get_weather()
    weather.cloudyness = 0
    weather.precipitation = 0
    weather.precipitation_deposits = 0
    weather.wind_intensity = 0
    weather.sun_azimuth_angle = 45
    weather.sun_altitude_angle = 100
    world.set_weather(weather)

    blueprint_library = world.get_blueprint_library()
    actor_list = list()

    print("> Creating Vehicles")
    print(">> Creating EGO vehicle")

    bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))
    transform = carla.Transform(carla.Location(x=-88.5, y=-160, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
    ego_vehicle = world.spawn_actor(bp, transform)
    actor_list.append(ego_vehicle)

    print(">>> Creating and attaching sensors to EGO")

    sensor_actors, sensor_labels = sensor_factory(world, ego_vehicle, sensor_list) 
    
    actor_list += sensor_actors

    print(">> Creating Opponents")

    opponent_actors = list()
    opponents_velocities = list()

    if use_case == USE_CASE_1:
        # 1 Opponent - Same Direction

        bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))
        transform = carla.Transform(carla.Location(x=-88.5, y=-120, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))
        opponents_velocities.append(sim_params["opponents_velocities"][0])
        
    elif use_case == USE_CASE_2:
        # 4 Opponents - Two way lane

        # Same lane
        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-88.5, y=-120, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        # Other Lane 3 vehicle

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-130, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0 + 180, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-60, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0 + 180, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=0, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0 + 180, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        opponents_velocities.append(sim_params["opponents_velocities"][0])
        opponents_velocities.append(-sim_params["opponents_velocities"][1])        
        opponents_velocities.append(-sim_params["opponents_velocities"][2])
        opponents_velocities.append(-sim_params["opponents_velocities"][3])

    elif use_case == USE_CASE_3:
        # 4 Opponents - Two lanes one way

        # Same lane
        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-88.5, y=-120, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        # Other Lane 3 vehicle

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-130, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-60, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=0, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        opponents_velocities.append(sim_params["opponents_velocities"][0])
        opponents_velocities.append(sim_params["opponents_velocities"][1])        
        opponents_velocities.append(sim_params["opponents_velocities"][2])
        opponents_velocities.append(sim_params["opponents_velocities"][3])

    elif use_case == USE_CASE_4:
        # 5 Opponents - Two Lanes one way, one parallel vehicle

        # Same lane
        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-88.5, y=-120, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        # Other Lane 3 vehicle

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-130, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-60, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=0, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        transform = carla.Transform(carla.Location(x=-85, y=-160, z=0.8), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
        opponent_actors.append(world.spawn_actor(bp, transform))

        opponents_velocities.append(sim_params["opponents_velocities"][0])
        opponents_velocities.append(sim_params["opponents_velocities"][1])        
        opponents_velocities.append(sim_params["opponents_velocities"][2])
        opponents_velocities.append(sim_params["opponents_velocities"][3])
        opponents_velocities.append(sim_params["opponents_velocities"][4])


    co_vehicle = opponent_actors[0]
    actor_list += opponent_actors

    try:
        # get all vehicles
        vehicles = [ego_vehicle] + opponent_actors # world.get_actors().filter('vehicle.*')
        vehicles_bb = [v for v in vehicles]

        time.sleep(2)
        
        print('> Creating data storage file', output_file_path)
        ds = DataStorageWriter(output_file_path, sensor_labels + ['bounding_box', 'vehicle_position', 'sensor_transform',
                                                                  'rect_camera1', 'rect_camera2', 'rect_camera3','rect_camera4',
                                                                  'rect_camera5', 'rect_camera6', 'rect_camera7','rect_camera8'])

        ds.write_matrix('bounding_box', 'extent', np.array([ClientSideBoundingBoxes._create_bb_points(vehicle) for vehicle in vehicles_bb]))
        ds.write_matrix('bounding_box', 'location',
                        np.array([[vehicle.bounding_box.location.x,
                                   vehicle.bounding_box.location.y,
                                   vehicle.bounding_box.location.z] for vehicle in vehicles_bb],dtype=np.float32))

        for idx, params in enumerate(sensor_list):
            ds.write_matrix('sensor_transform',
                            sensor_labels[idx],
                            np.array([params['x'], params['y'], params['z'], params['roll'], params['pitch'], params['yaw']], dtype=np.float32))
        
        print('> Starting simulation')

        with CarlaSyncMode(world, sensor_actors, fps=sim_params['fps']) as sync_mode:
            
            ego_vehicle.set_velocity(carla.Vector3D(0.0, sim_params['ego_velocity'], 0))

            for idx, opponent in enumerate(opponent_actors):
                opponent.set_velocity(carla.Vector3D(0.0, opponents_velocities[idx], 0))
            
            for idx in tqdm(range(int(sim_params['fps'])*60)):  # max 1min
            # for idx in range(int(sim_params['fps'])*60):  # max 1min
                data = sync_mode.tick(timeout=4.0)

                snapshot = data[0]
                frame = snapshot.frame
                ts = int(snapshot.timestamp.elapsed_seconds * 1e6)

                # store data
                sensors_data = data[1:]
                for idx, sensor_label in enumerate(sensor_labels):
                    sensor_data = sensors_data[idx]

                    if sensor_label.startswith('rgb'):
                        if save_rgb_as_jpeg:
                            ds.write_rgb_and_compact(sensor_label, ts, compute_data_buffer(sensor_data))
                        else:
                            ds.write_image(sensor_label, ts, compute_data_buffer(sensor_data))

                        if "10" in sensor_label:
                            bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(vehicles, sensor_actors[idx])
                            np_image = compute_data_buffer(sensor_data)
                            print("np_image", np_image.shape)
                            np_image2 = ClientSideBoundingBoxes.draw_bounding_boxes(np_image, bounding_boxes)

                            cv2.imshow("img", np_image2)
                            cv2.waitKey(1)

                    if sensor_label.startswith('depth'):
                        ds.write_matrix(sensor_label, ts, compute_depth_from_buffer(sensor_data))

                    if sensor_label.startswith('semseg'):
                        sensor_data.convert(carla.ColorConverter.CityScapesPalette)
                        ds.write_image(sensor_label, ts, compute_data_buffer(sensor_data))

                    if sensor_label.startswith('lidar'):
                        ds.write_lidar(sensor_label, ts, sensor_data)

                    if sensor_label.startswith('gnss'):
                        ds.write_gnss(sensor_label, ts, sensor_data)    

                # write vehicle position
                vehicle_pos = np.zeros((len(vehicles_bb), 6))
                for idx, vehicle in enumerate(vehicles_bb):
                    transform = vehicle.get_transform()
                    vehicle_pos[idx, 0] = transform.location.x
                    vehicle_pos[idx, 1] = transform.location.y
                    vehicle_pos[idx, 2] = transform.location.z
                    vehicle_pos[idx, 3] = transform.rotation.roll
                    vehicle_pos[idx, 4] = transform.rotation.pitch
                    vehicle_pos[idx, 5] = transform.rotation.yaw
                ds.write_matrix('vehicle_position', ts, vehicle_pos)

                if abs(ego_vehicle.get_location().y - co_vehicle.get_location().y) < 5:
                    print(">>> collision detected: stopping use case")
                    break

                if ego_vehicle.get_location().y > 140:
                    print(">>> ego vehicle is out of limits")
                    break

    finally:
        
        try:
            ds.close()
        except:
            pass

        print("> Cleaning Simulation")        
        for actor in actor_list:
            actor.destroy()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    output_file_path = "../dataset/data_usecase5.h5"
    use_case = USE_CASE_4
    save_rgb_as_jpeg = True

    run_use_case(use_case, output_file_path, SENSOR_LIST,
                 {"fps": 30,
                  "ego_velocity": 9.3,
                  "opponents_velocities": [4.3, 8.3, 8.3, 8.3, 8.3]},
                 save_rgb_as_jpeg)


# bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(vehicles, sensor_actors[5])
# np_image2 = ClientSideBoundingBoxes.draw_bounding_boxes(np_image, bounding_boxes)
                
# imgs = []
# for idx in range(8):
#     img = compute_data_buffer(data[idx*3+1])
#     img = cv2.resize(img, None, fx=0.35, fy=0.35)
#     # y, x = img.shape[0]//2, img.shape[1]//2
#     # img[y-3:y+3, x-3:x+3, :] = (0, 0, 255) 
#     imgs.append(img)

# img1 = np.concatenate((imgs[7], imgs[0], imgs[1], imgs[2], imgs[3]), axis=1)
# img2 = np.concatenate((imgs[7], imgs[6], imgs[5], imgs[4], imgs[3]), axis=1)
# img1 = np.concatenate((img1, img2), axis=0)

# cv2.imshow("img", np_image2)

# if cv2.waitKey(1) == ord('q'):
#     break