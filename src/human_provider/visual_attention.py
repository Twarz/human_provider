#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, ChannelFloat32, PointField
from human_provider.msg import HumanAttention
import tf2_ros
import tf2_py as tf2
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import PeoplePoseArray

import pybullet as p
import pybullet_data
from time import sleep, time
import math
import numpy as np

from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes, Situation, Property
from pyuwds.types.nodes import MESH, CAMERA
from pyuwds.uwds import MONITOR


ROOT_POSITION = np.array([0., 0., 0.]) # 1 meter above ground
ROOT_ORIENTATION = np.array([0., 0., 0., 1.])
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
    def __init__(self, resolution, position=ROOT_POSITION, orientation=ROOT_ORIENTATION):
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
        p.setAdditionalSearchPath('/home/twarz/catkin_ws/src/laas_objects/res/urdf')
        self.objects = {}
        self.object_names = {}

    def get_name(self, object_id):
        return self.object_names[object_id]

    def add_object(self, urdf_path, object_name, position=ROOT_POSITION, orientation=ROOT_ORIENTATION):
        try:
            self.objects[object_name] = p.loadURDF(urdf_path, position, orientation)
            self.object_names[self.objects[object_name]] = object_name
        except:
            print("ERROR, can't load: " + urdf_path)

    def update_object(self, urdf_path, object_name, position=ROOT_POSITION, orientation=ROOT_ORIENTATION):
        if object_name not in self.objects:
            self.add_object(urdf_path, object_name, position, orientation)
        else:
            p.resetBaseVelocity(self.objects[object_name], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            p.resetBasePositionAndOrientation(self.objects[object_name], position, orientation)

    def remove_object(self, object_name):
        p.removeBody(self.objects[object_name])
        del self.object_names[self.objects[object_name]]
        del self.objects[object_name]

    def get_pose_from_joint_id(self, object_name, joint_id):
        position, orientation = p.getLinkState(self.objects[object_name], joint_id)[:2]
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

        rays = p.rayTestBatch(ray_from, ray_to)
        
        point_clouds = {}

        for j, r in enumerate(rays):
            # object hit
            object_id = r[0]
            if (object_id >= 0):
                if object_id not in point_clouds.keys():
                    point_clouds[object_id] = []
                hit_position = r[3]
                point_clouds[object_id].append(list(hit_position) + [intensities[j]])
        return point_clouds

    def close(self):
        p.disconnect()

class VisualAttention(object):
    def __init__(self, gaze_topic_sub, point_cloud_topic):
        self.pc_pub = rospy.Publisher(point_cloud_topic, HumanAttention, queue_size=10)
        self.poses_sub = rospy.Subscriber(gaze_topic_sub, PeoplePoseArray, callback=self.callback)
        self.pybullet_env = PyBulletEnv(headless=HEADLESS)
        self.pybullet_env.add_object('plane.urdf', 'plane', [0, 0, -1])
        self.pybullet_env.add_object('human.urdf', 'human', [-2, 1, 1])
        self.cameras = {}
        

    def update_camera(self, cam_id, position, orientation):
        if cam_id in self.cameras.keys():
            self.cameras[cam_id].update(position, orientation)
        else:
            self.cameras[cam_id] = PyBulletVirtualCamera(RESOLUTION, position=position, orientation=orientation)

    def callback(self, people_pose_array):
        if not people_pose_array.poses:
            return 
        sigma = 0.4
        header = Header()
        header.frame_id = 'kinect2_nonrotated_link'
        header.stamp = people_pose_array.header.stamp
        
        for person in people_pose_array.poses:
            for limb_name, limb_pose in zip(person.limb_names, person.poses):
                person_id, limb_id = limb_name.split(':')
                #unique_id = int(person_id)*100 + int(20)
                person_id = '0'
                head_position = np.array([limb_pose.position.x, limb_pose.position.y, limb_pose.position.z])
                head_orientation = np.array([limb_pose.orientation.x, limb_pose.orientation.y, limb_pose.orientation.z, limb_pose.orientation.w])
                self.update_camera(person_id, head_position, head_orientation)
            
        for i, c in self.cameras.items():
            point_clouds = self.pybullet_env.compute_points_from_camera(c, SAMPLE_SIZE, MAX_RANGE, sigma*RESOLUTION[0]/2.)
            human_attention = HumanAttention()
            human_attention.human_id = i
            for pc_id, pc in point_clouds.items():
                pc_header = Header(stamp=header.stamp, frame_id=header.frame_id)
                pc2 = point_cloud2.create_cloud(pc_header, POINT_FIELDS, pc)
                human_attention.object_ids.append(self.pybullet_env.get_name(pc_id))
                human_attention.attentions.append(pc2)
            self.pc_pub.publish(human_attention)
        #p.removeAllUserDebugItems()
        
'''  
class VisualAttentionUWDS(ReconfigurableClient):
    def __init__(self, point_cloud_topic):
        self.pc_pub = rospy.Publisher(point_cloud_topic, PointCloud2, queue_size=10)
        self.pybullet_env = PyBulletEnv(headless=HEADLESS)
        self.cameras = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        super(VisualAttentionUWDS, self).__init__("visual_attention", MONITOR)
        
    def update_camera(self, cam_id, position, orientation):
        if cam_id in self.cameras.keys():
            self.cameras[cam_id].update(position, orientation)
        else:
            self.cameras[cam_id] = PyBulletVirtualCamera(RESOLUTION, position=position, orientation=orientation)
        return self.cameras[cam_id]

    def onChanges(self, world_name, header, invalidations):
        # first, remove deleted nodes
        for node_id in invalidations.node_ids_deleted:
            node = self.ctx.worlds()[world_name].scene().nodes()[node_id]
            if node.type == MESH:
                self.pybullet_env.remove_object(node_id)
            if node.type == CAMERA:
                del self.cameras[node_id]

        # then update the other ones
        for node_id in invalidations.node_ids_updated:
            node = self.ctx.worlds()[world_name].scene().nodes()[node_id]
            position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
            orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
            if node.type == MESH:
                urdf_path = node.name+".urdf"
                self.pybullet_env.update_object(urdf_path, node_id, position, orientation)
            elif node.type == CAMERA:
                self.update_camera(node_id, position, orientation)

        # finally, trigger raycasting
        p.stepSimulation()
        if self.cameras:
            sigma = 0.4
            for camera_id, camera in self.cameras.items():
                camera_frame = world_name + '/' + camera_id
                point_clouds = self.pybullet_env.compute_points_from_camera(camera, SAMPLE_SIZE, MAX_RANGE, sigma*RESOLUTION[0]/2.)
                for pc_id, pc in point_clouds.items():
                    pc_header = Header(stamp=header.stamp)
                    pc_header.frame_id = header.frame_id
                    pc2 = point_cloud2.create_cloud(pc_header, POINT_FIELDS, pc)
                    target_object_frame = world_name + '/' + self.pybullet_env.get_name(pc_id)
                    #new_cloud = self.tf_listener.transformPointCloud(target_object_frame, pc2)
                    #new_cloud = do_transform_cloud(pc2, trans)
                    self.pc_pub.publish(pc2)

    def onReconfigure(self, worlds):
        """
        """

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass
'''
'''
if __name__ == '__main__':
    rospy.init_node("visual_attention")
    point_cloud_topic = '/humans/visual_attention'
    visual_attention = VisualAttentionUWDS(point_cloud_topic)
    rospy.spin()
    visual_attention.pybullet_env.close()
''' 

if __name__ == '__main__':
    rospy.init_node("visual_attention") 
    gaze_topic_sub = "/humans/poses/3D/gaze"
    point_cloud_topic = '/humans/visual_attention'
    visual_attention = VisualAttention(gaze_topic_sub, point_cloud_topic)
    rospy.spin()
    visual_attention.pybullet_env.close()
