#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, ChannelFloat32, PointField
from human_visual_attention.msg import HumanAttention, HumanAttentionArray
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
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
SAMPLE_SIZE = 128
RESOLUTION = np.array([256, 256])
MAX_RANGE = 5

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
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(1)
        p.setAdditionalSearchPath('/home/twarz/catkin_ws/src/laas_objects/res/urdf')
        self.objects = {}
        self.object_names = {}

    def get_name(self, object_id):
        if object_id in self.object_names.keys():
            return self.object_names[object_id]
        return "nothing"

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
        if object_name in self.objects.keys():
            p.removeBody(self.objects[object_name])
            del self.object_names[self.objects[object_name]]
            del self.objects[object_name]

    def get_pose_from_joint_id(self, object_name, joint_id):
        position, orientation = p.getLinkState(self.objects[object_name], joint_id)[:2]
        return np.array(position), np.array(orientation)

    def compute_points_from_camera(self, camera, sample_size, max_range, sigma, sigma_pure):
        ray_from = []
        ray_to = []
        
        rand_x = np.random.normal(camera.center[0], sigma, sample_size)
        rand_x = np.insert(rand_x, 0, camera.center[0]) # insert center for center of attention

        rand_y = np.random.normal(camera.center[1], sigma, sample_size)
        rand_y = np.insert(rand_y, 0, camera.center[1]) # insert center for center of attention
        
        target_matrix = np.zeros((4, sample_size+1))
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

        rays = p.rayTestBatch(ray_from, ray_to)
        
        point_clouds = {}

        element_center_attention = rays[0][0] # get object id
        center_point = rays[0][3] # get hit position
        max_radius = 2.0
        intensity_scale = 1.0 - sigma_pure # scale between 0 and 1

        for r in rays[1:]:
            # object hit
            object_id = r[0]
            if (object_id >= 0):
                hit_position = r[3]
                d = min(math.sqrt(np.sum((np.array(center_point) - np.array(hit_position))**2)), max_radius) # clamped between 0 and 2
                intensity = 1.0 - (d / max_radius)
                #intensity *= intensity_scale
                intensity *= 0.05
                if object_id != element_center_attention:
                    intensity *= 0.75

                if object_id not in point_clouds.keys():
                    point_clouds[object_id] = []
                
                point_clouds[object_id].append(list(hit_position) + [intensity])
        
        return point_clouds, element_center_attention, center_point

    def close(self):
        p.disconnect()

'''
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
    def __init__(self, human_attentions_topic, headless):
        self.pc_pub = rospy.Publisher(human_attentions_topic, HumanAttentionArray, queue_size=10)
        self.pybullet_env = PyBulletEnv(headless=headless)
        self.cameras = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pybullet_env.add_object('plane.urdf', 'plane', [0, 0, -1])

        # add PR2 robot
        self.pybullet_env.add_object('pr2_base.urdf', 'pr2_base', [-1, 0, -0.9])
        self.pybullet_env.add_object('pr2_body.urdf', 'pr2_body', [-1, 0, -0.45])
        self.pybullet_env.add_object('pr2_head.urdf', 'pr2_head', [-1, 0, 0.15])
        self.pybullet_env.add_object('pr2_arm.urdf', 'pr2_larm', [-0.65, 0.15, -0.30])
        self.pybullet_env.add_object('pr2_arm.urdf', 'pr2_rarm', [-0.65, -0.15,-0.30])
        self.pybullet_env.add_object('pr2_grip.urdf', 'pr2_lgrip', [-0.65, 0.15, -0.1])
        self.pybullet_env.add_object('pr2_grip.urdf', 'pr2_rgrip', [-0.65, -0.15,-0.1])

        # add Pepper robot
        self.pybullet_env.add_object('pepper_base.urdf', 'pepper_base', [-1, 1, -0.9])
        self.pybullet_env.add_object('pepper_body.urdf', 'pepper_body', [-1, 1, -0.55])
        self.pybullet_env.add_object('pepper_tablet.urdf', 'pepper_tablet', [-0.8, 1, -0.55])
        self.pybullet_env.add_object('pepper_head.urdf', 'pepper_head', [-1, 1, -0.2])
        self.pybullet_env.add_object('pepper_arm.urdf', 'pepper_larm', [-1, 1.2, -0.5])
        self.pybullet_env.add_object('pepper_arm.urdf', 'pepper_rarm', [-1, 0.8,-0.5])
        self.pybullet_env.add_object('pepper_hand.urdf', 'pepper_lhand', [-1, 1.2, -0.6])
        self.pybullet_env.add_object('pepper_hand.urdf', 'pepper_rhand', [-1, 0.8,-0.6])

        super(VisualAttentionUWDS, self).__init__("visual_attention", MONITOR)
        
    def update_camera(self, cam_id, position, orientation):
        if cam_id in self.cameras.keys():
            self.cameras[cam_id].update(position, orientation)
        else:
            self.cameras[cam_id] = PyBulletVirtualCamera(RESOLUTION, position=position, orientation=orientation)
        return self.cameras[cam_id]

    def remove_camera(self, node_id):
        if node_id in self.cameras.keys():
            del self.cameras[node_id]

    def onChanges(self, world_name, header, invalidations):
        # first, remove deleted nodes
        for node_id in invalidations.node_ids_deleted:
            self.pybullet_env.remove_object(node_id)
            self.remove_camera(node_id)

        # then update the other ones
        for node_id in invalidations.node_ids_updated:
            node = self.ctx.worlds()[world_name].scene().nodes()[node_id]
            position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
            orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
            if node.type == MESH:
                urdf_path = node.name+".urdf"
                self.pybullet_env.update_object(urdf_path, node_id, position, orientation)
            elif node.type == CAMERA:
                print(node_id)
                self.pybullet_env.update_object('human_head.urdf', node_id, position, orientation)
                self.pybullet_env.update_object('human_body.urdf', node_id+'_body', position)
                self.update_camera(node_id, position, orientation)

        # finally, trigger raycasting
        p.stepSimulation()
        if self.cameras:
            attentions = HumanAttentionArray()
            attentions.header.frame_id = 'webcam_link'
            attentions.header.stamp = header.stamp
            sigma = 0.4
            for camera_id, camera in self.cameras.items():
                camera_frame = world_name + '/' + camera_id
                point_clouds, center_element, center_point = self.pybullet_env.compute_points_from_camera(camera, SAMPLE_SIZE, MAX_RANGE, sigma*RESOLUTION[0]/2., sigma)

                human_attention = HumanAttention()
                human_attention.human_id = camera_id
                human_attention.element_of_attention = self.pybullet_env.get_name(center_element)
                human_attention.point_of_attention.x = center_point[0]
                human_attention.point_of_attention.y = center_point[1]
                human_attention.point_of_attention.z = center_point[2]

                for pc_id, pc in point_clouds.items():
                    pc_header = Header(stamp=header.stamp, frame_id=header.frame_id)
                    pc2 = point_cloud2.create_cloud(pc_header, POINT_FIELDS, pc)
                    human_attention.element_ids.append(self.pybullet_env.get_name(pc_id))
                    human_attention.elements.append(pc2)
                    '''
                    human_attention.object_ids.append(self.pybullet_env.get_name(pc_id))
                    target_object_frame = world_name + '/' + self.pybullet_env.get_name(pc_id)

                    node = self.ctx.worlds()[world_name].scene().nodes()[self.pybullet_env.get_name(pc_id)]
                    # transform = self.tf_buffer.lookup_transform(target_object_frame, pc_header.frame_id, rospy.Time())
                    t = TransformStamped()
                    t.header.stamp = pc_header.stamp
                    t.header.frame_id = 'map'
                    t.child_frame_id = target_object_frame
                    t.transform.translation = node.position.pose.position
                    t.transform.rotation = node.position.pose.orientation
                    
                    new_pc = do_transform_cloud(pc2, t)
                    new_pc.header.frame_id = t.child_frame_id
                    #ew_cloud = self.tf_listener.transformPointCloud(target_object_frame, pc2)
                    human_attention.attentions.append(new_pc)
                    '''
                    #self.pc_pub.publish(pc2)
                attentions.humans.append(human_attention)
            self.pc_pub.publish(attentions)

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

if __name__ == '__main__':
    rospy.init_node("visual_attention")
    human_attentions_topic = '/humans/visual_attention'
    headless = rospy.get_param("~headless", True)
    visual_attention = VisualAttentionUWDS(human_attentions_topic, headless)
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
'''