import carla
import math
import random
import time
import queue
import numpy as np
import cv2

EDGES = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

class BoundingBox():
    '''
    EXAMPLE USAGE:
        
    from boundingboxes import BoundingBox
    
    blueprint_rgb = blueprint_library.find('sensor.camera.rgb')
    BB= BoundingBox(blueprint_rgb)
    
    bbout= BB.get(world.get_actors().filter('*vehicle*'), ego_vehicle, camera_rgb)
    '''
    def __init__(self, camera_bp):
        
        # Get the attributes from the camera
        self.image_w = camera_bp.get_attribute("image_size_x").as_int()
        self.image_h = camera_bp.get_attribute("image_size_y").as_int()
        fov = camera_bp.get_attribute("fov").as_float()
        
        # Calculate the camera projection matrix to project from 3D -> 2D
        self.K = build_projection_matrix(self.image_w, self.image_h, fov)
        
    def get(self, actor_list, vehicle, camera):
        
        result= []
    
        # Get the camera matrix 
        world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    
        for npc in actor_list:
    
            # Filter out the ego vehicle
            if npc.id == vehicle.id:
                continue
    
            bb = npc.bounding_box
            dist = npc.get_transform().location.distance(vehicle.get_transform().location)

            # Filter for the vehicles within 50m
            if dist < 50:

            # Calculate the dot product between the forward vector
            # of the vehicle and the vector between the vehicle
            # and the other vehicle. We threshold this dot product
            # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = vehicle.get_transform().get_forward_vector()
                ray = npc.get_transform().location - vehicle.get_transform().location

                if forward_vec.dot(ray) > 1:
                    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                    x_max = -10000
                    x_min = 10000
                    y_max = -10000
                    y_min = 10000

                    for vert in verts:
                        p = get_image_point(vert, self.K, world_2_camera)
                        # Find the rightmost vertex
                        if p[0] > x_max:
                            x_max = min([p[0], self.image_w])
                        # Find the leftmost vertex
                        if p[0] < x_min:
                            x_min = max([p[0], 0])
                        # Find the highest vertex
                        if p[1] > y_max:
                            y_max = min([p[1], self.image_h])
                        # Find the lowest  vertex
                        if p[1] < y_min:
                            y_min = max([p[1], 0])
                    result.append([npc.id, x_min, y_min, x_max, y_max, 1, 0])
        return np.array(result)
        
    
        