#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
#import cv2
import time
import numpy as np
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

#colorcode converter
# def hex_to_bgr(hex_code):
#     hex_code = hex_code.lstrip('#')
#     return tuple(int(hex_code[i:i+2], 16) for i in (4, 2, 0))
# target_hex_code = "#01018e" 
# target_bgr = hex_to_bgr(target_hex_code)

#######################################################################

def main():
    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.*')),
            start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(True)

        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=4, z=3), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)

        camera_semseg = world.spawn_actor(
            blueprint_library.find('sensor.camera.semantic_segmentation'),
            carla.Transform(carla.Location(x=4, z=3), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_semseg)

        camera_depth = world.spawn_actor(
            blueprint_library.find('sensor.camera.depth'),
            carla.Transform(carla.Location(x=4, z=3), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_depth)

        # transform = random.choice(world.get_map().get_spawn_points())

        # transform.location += carla.Location(x=40, y=-3.2)
        # transform.rotation.yaw = -180.0
        # for _ in range(0, 50):
        #     transform.location.x += 8.0

        #     bp = random.choice(blueprint_library.filter('vehicle'))

        #     # This time we are using try_spawn_actor. If the spot is already
        #     # occupied by another object, the function will return None.
        #     npc = world.try_spawn_actor(bp, transform)
        #     if npc is not None:
        #         actor_list.append(npc)
        #         npc.set_autopilot(True)
        #         print('created %s' % npc.type_id)
        #Create a synchronous mode context.
        ###################################################################################
        #traffic_manager

        # tm = client.get_trafficmanager(2000)

        # tm_port = tm.get_port()
        # for v in vehicles_list:
        #     v.set_autopilot(True,tm_port)
        #     traffic_manager = client.get_trafficmanager(args.tm-port)
        # tm_port = traffic_manager.get_port()
        # batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True,tm_port)))
        # traffic_manager.global_percentage_speed_difference(30.0)

        # tm = client.get_trafficmanager(2000)
        # tm_port = tm.get_port()
        # for v in my_vehicles:
        #     v.set_autopilot(True,tm_port)
        # danger_car = my_vehicles[0]
        # tm.ignore_lights_percentage(danger_car,100)
        # tm.distance_to_leading_vehicle(danger_car,0)
        # tm.vehicle_percentage_speed_difference(danger_car,-20)

        # tm = client.get_trafficmanager(2000)
        # tm_port = tm.get_port()
        # for v in my_vehicles:
        #     v.set_autopilot(True,tm_port)
        # danger_car = my_vehicles[0]
        # tm.global_distance_to_leading_vehicle(5)
        # tm.global_percentage_speed_difference(80)
        # for v in my_vehicles: 
        #     tm.auto_lane_change(v,False)

        # tm = client.get_trafficmanager(2000)
        # for actor in my_vehicles:
        #     tm.update_vehicle_lights(actor, True)


        with CarlaSyncMode(world, camera_rgb, camera_semseg, camera_depth, fps=30) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                # Advance the simulation and wait for the data.
                snapshot, image_rgb, image_semseg, image_depth= sync_mode.tick(timeout=2.0)

                # Choose the next waypoint and update the car location.
                waypoint = random.choice(waypoint.next(1.5))
                vehicle.set_transform(waypoint.transform)

                image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                image_depth.convert(carla.ColorConverter.Depth)

                # array = np.frombuffer(image_semseg.raw_data, dtype=np.dtype("uint8"))
                # array = np.reshape(array, (image_semseg.height, image_semseg.width, 4))
                # array = array[:, :, ::-1]  
                # opencv_image = array 

                # target_bgr=(142, 1, 1)

                # # Find matching pixels
                # matching_pixels = np.all(opencv_image == target_bgr, axis=2)

                # total_pixels = opencv_image.shape[0] * opencv_image.shape[1]
                # percentage = (matching_pixels / total_pixels) * 100

                # print(percentage)

                # if percentage>1:
                #     #saving images to disk  
                #     image_semseg.save_to_disk('_out/data_out/sync_data/semseg/%08d.png' % snapshot.frame)
                #     image_depth.save_to_disk('_out/data_out/sync_data/depth/%08d.png' % snapshot.frame)
                #     image_rgb.save_to_disk('_out/data_out/sync_data/rgb/%08d.png' % snapshot.frame)

                image_semseg.save_to_disk('_out/data_out/sync_data/semseg/%08d.png' % snapshot.frame)
                image_depth.save_to_disk('_out/data_out/sync_data/depth/%08d.png' % snapshot.frame)
                image_rgb.save_to_disk('_out/data_out/sync_data/rgb/%08d.png' % snapshot.frame)

                # Draw the display.
                draw_image(display, image_rgb)
                draw_image(display, image_semseg, blend=True)
                draw_image(display, image_depth, blend=True)

                time.sleep(3)

                

                display.blit(
                    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    (8, 28))
                pygame.display.flip()

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicle])

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')