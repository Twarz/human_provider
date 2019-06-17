#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, ChannelFloat32, PointField
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import PeoplePoseArray

import pybullet as p
from time import sleep, time
import math
import numpy as np


DEFAULT_POSITION = np.array([0., 0., 1.]) # 1 meter above ground
DEFAULT_ORIENTATION = np.array([0., 0., 0., 1.])
DEFAULT_AXIS = np.eye(3)
SAMPLE_SIZE = 256
HEADLESS = True
RESOLUTION = np.array([256, 256])
MAX_RANGE = 10

POINT_FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]

class PyBulletVirtualCamera(object):
    def __init__(self, resolution, position=DEFAULT_POSITION, orientation=DEFAULT_ORIENTATION):
        self.fov = 60.0
        self.aspect = 1.3333
        self.clipnear = 0.3
        self.clipfar = 100.0
        self.resolution = resolution
        self.inverse_resolution = 1./self.resolution
        self.center = self.resolution * 0.5

        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.clipnear, self.clipfar)
        self.update(position, orientation)

    def update(self, position, orientation):
        self.position = position
        self.orientation = orientation

        rotation_matrix = np.array(p.getMatrixFromQuaternion(self.orientation)).reshape(3, 3)
        rotated_frame = rotation_matrix.dot(DEFAULT_AXIS)

        forward_vector = rotated_frame[:,0]
        roll_vector = rotated_frame[:,1]
        yaw_vector = rotated_frame[:,2]

        #self.view_matrix = p.computeViewMatrix(head_position, head_position + self.clipnear * camera_vector, horiz_vector)

        dHor = yaw_vector * self.inverse_resolution[0]
        dVer = roll_vector * self.inverse_resolution[1]
        self.forward = np.add(self.position, forward_vector)
        self.trans_mat = np.array([yaw_vector, roll_vector, dHor, dVer]).transpose()


class PyBulletEnv(object):
    def __init__(self, headless=True):
        if headless:
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(1)
        self.object = {}

    def add_object(self, urdf_path, object_name, position):
        self.object[object_name] = p.loadURDF(urdf_path, position)

    def get_pose_from_joint_id(self, object_name, joint_id):
        position, orientation = p.getLinkState(self.object[object_name], joint_id)[:2]
        return np.array(position), np.array(orientation)


    def compute_points_from_camera(self, camera, sample_size, max_range, sigma):
        ray_from = []
        ray_to = []
        intensities = []
        
        rand_x = np.random.normal(camera.center[0], sigma, sample_size)
        rand_y = np.random.normal(camera.center[1], sigma, sample_size)
        
        target_matrix = np.zeros((4, sample_size))
        target_matrix[0,:] = -0.5
        target_matrix[1,:] = 0.5
        target_matrix[2,:] = rand_x
        target_matrix[3,:] = -rand_y

        ortho = camera.trans_mat.dot(target_matrix)
        rayTo = (ortho + (camera.forward-camera.position)[:,np.newaxis]) * max_range

        for s in range(sample_size):
            target = rayTo[:,s]
            ray_from.append(camera.position)
            ray_to.append(target)
            intensities.append((1-math.sqrt(((rand_x[s]-camera.center[0])/RESOLUTION[0])**2 + ((rand_y[s]-camera.center[1])/RESOLUTION[1])**2))/10.)
            #intensities.append(0.1)

        rays = p.rayTestBatch(ray_from, ray_to)
        
        position_intensity_object = []

        for j, r in enumerate(rays):
            object_id = r[0]

            if (object_id < 0):
                # nothing hit
                #p.addUserDebugLine(ray_from[j], ray_to[j], [1, 0, 0])
                pass
            else:
                # object hit
                hit_position = r[3]
                position_intensity_object.append(list(hit_position) + [intensities[j]])
                #p.addUserDebugLine(ray_from[j], hit_position, [0, 1, 0])

        return position_intensity_object

    def close(self):
        p.disconnect()

class VisualAttention(object):
    def __init__(self, gaze_topic_sub, point_cloud_topic):
        self.pc_pub = rospy.Publisher(point_cloud_topic, PointCloud2, queue_size=10)
        self.poses_sub = rospy.Subscriber(gaze_topic_sub, PeoplePoseArray, callback=self.callback)
        self.pybullet_env = PyBulletEnv(headless=HEADLESS)
        self.pybullet_env.add_object('plane.urdf', 'plane', [0, 0, -1])
        self.pybullet_env.add_object('human.urdf', 'human', [-2, 1, 1])
        self.camera = {}
        

    def get_camera(self, cam_id, position, orientation):
        if cam_id in self.camera.keys():
            self.camera[cam_id].update(position, orientation)
        else:
            self.camera[cam_id] = PyBulletVirtualCamera(RESOLUTION, position=position, orientation=orientation)
        return self.camera[cam_id]

    def callback(self, people_pose_array):
        if not people_pose_array.poses:
            return 
        sigma = 0.4
        header = Header()
        header.frame_id = 'kinect2_nonrotated_link'
        header.stamp = people_pose_array.header.stamp
        
        for person in people_pose_array.poses:
            position_intensity_object = []
            for limb_name, limb_pose in zip(person.limb_names, person.poses):
                person_id, limb_id = limb_name.split(':')
                #unique_id = int(person_id)*100 + int(20)
                head_position = np.array([limb_pose.position.x, limb_pose.position.y, limb_pose.position.z])
                head_orientation = np.array([limb_pose.orientation.x, limb_pose.orientation.y, limb_pose.orientation.z, limb_pose.orientation.w])
                camera = self.get_camera(person_id, head_position, head_orientation)
                p.stepSimulation()
                position_intensity_object = self.pybullet_env.compute_points_from_camera(camera, SAMPLE_SIZE, MAX_RANGE, sigma*RESOLUTION[0]/2.)
            
            pc2 = point_cloud2.create_cloud(header, POINT_FIELDS, position_intensity_object)
            self.pc_pub.publish(pc2)
        p.removeAllUserDebugItems()
        
                
        
if __name__ == '__main__':
    rospy.init_node("visual_attention_node") 
    gaze_topic_sub = "/humans/poses/3D/gaze"
    point_cloud_topic = '/humans/visual_attention'
    visual_attention = VisualAttention(gaze_topic_sub, point_cloud_topic)
    rospy.spin()
    visual_attention.pybullet_env.close()

'''
if __name__ == '__main__':

    # load the env
    pybullet_env = PyBulletEnv(headless=HEADLESS)
    pybullet_env.add_object('plane.urdf', 'plane', [0, 0, 0])
    pybullet_env.add_object('human.urdf', 'human', [0, 0, 1])

    # init camera position from head pose 
    head_position, head_orientation = pybullet_env.get_pose_from_joint_id('human', HEAD_ID)
    camera = PyBulletVirtualCamera(RESOLUTION, position=head_position, orientation=head_orientation)

    start = time()
    sigma = 0.5

    # main loop
    for i in range(NUM_STEPS):
        p.stepSimulation()
        head_position, head_orientation = pybullet_env.get_pose_from_joint_id('human', HEAD_ID)
        camera.update(head_position, head_orientation)
        points, object_ids = pybullet_env.compute_points_from_camera(camera, SAMPLE_SIZE, MAX_RANGE, sigma*RESOLUTION[0]/2.)
        p.removeAllUserDebugItems()
        #sleep(1/120.)
    print(time()-start)
    pybullet_env.close()
'''