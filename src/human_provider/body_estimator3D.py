#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
import scipy
import tf
import uuid

from image_geometry import PinholeCameraModel
from kalman_stabilizer import Stabilizer
from SequentialTracker import SequentialTracker
from GenericTracker import TrackedElement

class TrackedBody(TrackedElement):
    def __init__(self, body):
        super(TrackedBody, self).__init__()
        self.body = body

    def compute_distance(self, other_body):
        common_parts = list(set(self.body.keys()) & set(other_body.body.keys()))
        if not common_parts:
            return 1e+10 #float("inf")
        np_parts = np.concatenate([self.body[p] for p in common_parts])
        np_other_parts = np.concatenate([other_body.body[p] for p in common_parts])
        return np.sqrt(np.sum(np.sum(np.square(np_parts - np_other_parts), axis=0)))
        
class BodyEstimator3D(object):
    def __init__(self, camera_info):
        self.pinhole_camera_model = PinholeCameraModel()
        self.pinhole_camera_model.fromCameraInfo(camera_info)
        self.body_tracker = SequentialTracker()
        self.kalman_filter = {} # TODO
        self.image_resolution = np.array([camera_info.width, camera_info.height])
        self.bound_x = [0, self.image_resolution[0] - 1]
        self.bound_y = [0, self.image_resolution[1] - 1]
        self.kernel_size = 4
        self.part_id_to_exclude = [8,9,10, 11,12,13, 14,15,16,17, 18]

    def estimate(self, bodies2D, np_depth_img):
        filtered_bodies2D = [self.__filter_body_parts(body2D) for body2D in bodies2D]
        
        tracked_bodies3D = self.__project_body_on_disparity(filtered_bodies2D, np_depth_img)

        # track the bodies regarding the last callback
        self.body_tracker.track(tracked_bodies3D)
        tracked_bodies3D = self.body_tracker.get_tracked_elements()

        # smooth the poses
        #stab_bodies3D = self.kalman_filter.stabilize(tracked_bodies3D)
        return tracked_bodies3D

    def reset(self):
        self.body_tracker.clear_elements()

    def __filter_body_parts(self, body):
        return {part_id:part for (part_id, part) in body.items() if part_id not in self.part_id_to_exclude}

    def __project_body_on_disparity(self, bodies, np_depth_img):
        tracked_bodies3D = []
        for body in bodies:
            body_dict = {}
            for part_id, part in body.items():
                np_pos3D = self.__project_point2D_on_disparity(part[:2], np_depth_img)
                if np_pos3D is not None:
                    body_dict[part_id] = np.array([np_pos3D, part[2]])
            tracked_bodies3D.append(TrackedBody(body_dict))
        return tracked_bodies3D

    def __project_point2D_on_disparity(self, np_normalized_point, np_depth_img):
        np_pos = np.multiply(np_normalized_point, self.image_resolution)
        np_rect_pos = np.array(self.pinhole_camera_model.rectifyPoint((np_pos[0], np_pos[1])))
        np_vector3D = np.array(self.pinhole_camera_model.projectPixelTo3dRay((np_rect_pos[0], np_rect_pos[1])))
        
        np_clipped_rect_pos = np.clip(np_rect_pos, [0, 0], self.image_resolution)
        x, y = int(np_clipped_rect_pos[0]), int(np_clipped_rect_pos[1])
        bloc = np_depth_img[y-self.kernel_size:y+self.kernel_size,x-self.kernel_size:x+self.kernel_size] # TODO: check out of bounds
        nzeros = np.nonzero(bloc)
        if len(nzeros[0]) > 0:
            depth = np.min(bloc[nzeros]) / 1000.0 # scale
            return np_vector3D*depth

        return None

class Stabilizer3DPoint(object):

    def __init__(self, x, y, z):
        self.x_value = x
        self.y_value = y
        self.z_value = z
        self.x_stab = Stabilizer(state_num=2, measure_num=1, cov_process=0.1, cov_measure=0.1)
        self.x_stab.state[0] = self.x_value
        self.x_stab.state[1] = self.x_value
        self.y_stab = Stabilizer(state_num=2, measure_num=1, cov_process=0.1, cov_measure=0.1)
        self.y_stab.state[0] = self.y_value
        self.y_stab.state[1] = self.y_value
        self.z_stab = Stabilizer(state_num=2, measure_num=1, cov_process=0.1, cov_measure=0.1)
        self.z_stab.state[0] = self.z_value
        self.z_stab.state[1] = self.z_value

    def update(self, x, y, z):
        self.x_stab.update([x])
        self.x_value = self.x_stab.state[0]
        self.y_stab.update([y])
        self.y_value = self.y_stab.state[0]
        self.z_stab.update([z])
        self.z_value = self.z_stab.state[0]

    def state(self):
        return self.x_value, self.y_value, self.z_value

class HumanBody3D(object):
    """
    """
    def stabilize(self, body_id, part_id, x, y, z):
        if part_id in self.stabilizers[body_id].keys():
            self.stabilizers[body_id][part_id].update(x, y, z)
        else:
            self.stabilizers[body_id][part_id] = Stabilizer3DPoint(x, y, z)
        return self.stabilizers[body_id][part_id].state()
