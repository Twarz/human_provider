#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, ChannelFloat32, PointField, JointState
from human_visual_attention.msg import HumanAttention, HumanAttentionArray
import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf.transformations import quaternion_multiply, euler_from_quaternion
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Quaternion
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

from enum import Enum

import matplotlib.pyplot as plt


from image_geometry import PinholeCameraModel
pinhole_camera_model = PinholeCameraModel()
#pinhole_camera_model.fromCameraInfo(camera_info)


ROOT_POSITION = np.array([0., 0., 0.]) # 1 meter above ground
ROOT_ORIENTATION = np.array([0., 0., 0., 1.])
DEFAULT_AXIS = np.eye(3)
SAMPLE_SIZE = 256
RESOLUTION = np.array([128, 128])
MAX_RANGE = 20

POINT_FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]

RAY_COLOR = [0, 1, 0]
RAY_COLOR_CENTER = [1, 0, 0]

class Speed(Enum):
    STATIC = 1.0
    SLOW = 0.75
    MEDIUM = 0.5
    HIGH = 0.25
    TOO_HIGH = 0.0
    NONE = 0.0


conj = lambda q: Quaternion(x=-q.x,y=-q.y,z=-q.z,w=q.w)

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
        self.speed = 1.0
        self.orientation = None
        self.position = None
        self.update(position, orientation)
        

    def update(self, position, orientation, dt=0.0):
        print('dt: ' + str(dt))
        if self.orientation is not None:
            euler_before = euler_from_quaternion(orientation)
            euler_after = euler_from_quaternion(self.orientation)
            print()
            print(euler_before)
            print(euler_after)
            euler_diff = (np.array(euler_before)-np.array(euler_after))# / dt
            euler_angular_speed = np.sqrt(np.sum(np.square(np.array(euler_diff))))
            print('angular speed euler: ' + str(euler_diff))
            print('mean speed euler: ' + str(euler_angular_speed))

            '''
            q_diff = np.array(orientation) - np.array(self.orientation)
            q_conj = np.array([-self.orientation[0], -self.orientation[1], -self.orientation[2], self.orientation[3]])
            angular_velocity = 2 * quaternion_multiply(q_diff, q_conj) / dt
            mean_speed = np.sqrt(np.sum(np.square(np.array(angular_velocity))))
            print('angular velocity: ' + str(angular_velocity))
            print('mean speed: ' + str(mean_speed))
            '''
            '''
            if mean_speed < 0.2:
                self.speed = Speed.STATIC
            elif mean_speed < 1.0:
                self.speed = Speed.SLOW
            elif mean_speed < 2.0:
                self.speed = Speed.MEDIUM
            elif mean_speed < 3.0:
                self.speed = Speed.HIGH
            else:
                self.speed = Speed.TOO_HIGH
            '''

            self.speed = euler_angular_speed


        self.position = position
        self.orientation = orientation

        rotation_matrix = np.array(p.getMatrixFromQuaternion(self.orientation)).reshape(3, 3).T
        print(rotation_matrix)
        rotated_frame = rotation_matrix.dot(DEFAULT_AXIS)

        self.forward_vector = rotated_frame[:,0]
        self.roll_vector = rotated_frame[:,1]
        self.yaw_vector = rotated_frame[:,2]

        #self.view_matrix = p.computeViewMatrix(head_position, head_position + self.clipnear * camera_vector, horiz_vector)

        dHor = self.yaw_vector * self.inverse_resolution[0]
        dVer = self.roll_vector * self.inverse_resolution[1]
        #self.forward = np.add(self.position, self.forward_vector)
        self.trans_mat = np.array([self.yaw_vector, self.roll_vector, dHor, dVer]).transpose()

    def get_depth_image(self, width, height, max_range):
        camera_target = np.add(self.position, self.forward_vector * max_range) 
        p.addUserDebugLine(self.position, [camera_target[0], camera_target[1], camera_target[2]], RAY_COLOR)
        view_matrix = p.computeViewMatrix(self.position, camera_target, self.yaw_vector)

        camera_image = p.getCameraImage(
                width,
                height,
                view_matrix,
                self.projection_matrix,
                renderer=p.ER_TINY_RENDERER)
                #renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                #flags=pybullet.ER_NO_SEGMENTATION_MASK,
                #physicsClientId=self.physics_client)

        #depth_buffer_tiny = np.reshape(camera_image[3], (height, width))
        #print(depth_buffer_tiny)
        depth_tiny = (self.clipfar * self.clipnear) / (self.clipfar - (self.clipfar - self.clipnear) * camera_image[3])
        #depth_tiny = camera_image[3]
        print(depth_tiny)
        #depth_tiny /= 1000

        inv_view_matrix = np.linalg.inv(np.reshape(view_matrix, (4,4)))
        
        projection_matrix_44 = np.reshape(self.projection_matrix, (4,4)).T
        fx = projection_matrix_44[0, 0] #509.015381 projection_matrix_44[0, 0]
        fy = projection_matrix_44[1, 1] #320.675346 projection_matrix_44[1, 1]
        cx = projection_matrix_44[0, 2] #508.683267 # projection_matrix_44[0, 2]
        cy = projection_matrix_44[1, 2] #236.130072 # projection_matrix_44[1, 2]

        point_clouds = {}
        #P3D.x = (x_d - cx_d) * depth(x_d,y_d) / fx_d
        #P3D.y = (y_d - cy_d) * depth(x_d,y_d) / fy_d
        #P3D.z = depth(x_d,y_d)
        pt2d = np.array([-1, -1, 1, 1])
        #pt3d = np.dot(pt2d, np.linalg.inv(projection_matrix_44))
        pt3d = np.dot(pt2d, inv_view_matrix)
        p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)

        pt2d = np.array([-1, 1, 1, 1])
        #pt3d = np.dot(pt2d, np.linalg.inv(projection_matrix_44))
        pt3d = np.dot(pt2d, inv_view_matrix)
        p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)

        pt2d = np.array([1, -1, 1, 1])
        #pt3d = np.dot(pt2d, np.linalg.inv(projection_matrix_44))
        pt3d = np.dot(pt2d, inv_view_matrix)
        p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)

        pt2d = np.array([1, 1, 1, 1])
        #pt3d = np.dot(pt2d, np.linalg.inv(projection_matrix_44))
        pt3d = np.dot(pt2d, inv_view_matrix)
        p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)

        pt2d = np.array([0, 0, 1, 1])
        #pt3d = np.dot(pt2d, np.linalg.inv(projection_matrix_44))
        pt3d = np.dot(pt2d, inv_view_matrix)
        p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)

        '''
        for v in np.arange(-1., 1., 2./height):
            for u in np.arange(-1., 1., 2./width):
                object_id = camera_image[4][int(v*height), int(u*width)]
                pt2d = np.array([u, v, 1, 1])
                pt3d = np.dot(pt2d, inv_view_matrix)[:3]

                pt3d_norm = pt3d / (pt3d**2).sum()**0.5
                pt3d = pt3d_norm * depth_tiny[int(v*height), int(u*width)]

                #pt3d = pt3d * depth_tiny[int(v*depth_tiny.shape[0]), int(u*depth_tiny.shape[1])]
                #p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)
                if u == 0 and v == 0:
                    center_element = object_id
                    center_point = pt3d
                    p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR_CENTER)
                    continue
                else:
                    pass
                    if object_id >= -1:
                        p.addUserDebugLine(self.position, [pt3d[0], pt3d[1], pt3d[2]], RAY_COLOR)

                if object_id not in point_clouds.keys():
                    point_clouds[object_id] = []
                
                point_clouds[object_id].append(list(pt3d[:3]) + [1])

        #depth_image *= 1000
        #frame = depth_tiny.astype(np.uint16)
        return point_clouds, center_element, center_point
        '''
        for v in range(height):
            v_norm = v / float(depth_tiny.shape[0])
            for u in range(width):
                u_norm = u / float(depth_tiny.shape[1])
                object_id = camera_image[4][v, u]
                x = (u - 8.) * (-depth_tiny[v, u]) / 16.
                y = (v - 8.) * (-depth_tiny[v, u]) / 16.
                norm = math.sqrt(x*x + y*y + 1)
                #x /= norm
                #y /= norm
                z = -depth_tiny[v, u] # 1.0 / norm
                
                point_3D = np.dot(np.array([x, y, z, 1]), inv_view_matrix)[:3]

                if u == width / 2. and v == height / 2.:
                    center_element = object_id
                    center_point = point_3D
                    p.addUserDebugLine(self.position, [point_3D[0], point_3D[1], point_3D[2]], RAY_COLOR_CENTER)
                    continue
                else:
                    #pass
                    p.addUserDebugLine(self.position, [point_3D[0], point_3D[1], point_3D[2]], RAY_COLOR)

                if object_id not in point_clouds.keys():
                    point_clouds[object_id] = []
                
                point_clouds[object_id].append(list(point_3D) + [1])

        #depth_image *= 1000
        #frame = depth_tiny.astype(np.uint16)
        return point_clouds, center_element, center_point

class RobotListener(object):
    def __init__(self, robot_urdf):
        self.global_frame_id = 'map'
        self.bullet_node_id_map = {}
        # Initialize tf2 buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.robot_urdf = robot_urdf
        self.base_frame_id = "base_footprint"
        sleep(1.)
        _, t, q = self.get_transform_from_tf2(self.global_frame_id, self.base_frame_id)
        self.bullet_node_id_map[self.base_frame_id] = p.loadURDF(self.robot_urdf, t, q)
        self.init_robot_joint_map()
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.update_robot_joints)
        

    def init_robot_joint_map(self):
        self.simulator_joint_id_map = {}
        for i in range(0, p.getNumJoints(self.bullet_node_id_map[self.base_frame_id])):
            info = p.getJointInfo(self.bullet_node_id_map[self.base_frame_id], i)
            joint_name = info[1]
            self.simulator_joint_id_map[joint_name] = i

    def get_transform_from_tf2(self, source_frame, target_frame):
        try:
            trans = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            rx = trans.transform.rotation.x
            ry = trans.transform.rotation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            return True, [x, y, z], [rx, ry, rz, rw]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]

    def update_robot_base(self):
        base_link_sim_id = self.bullet_node_id_map[self.base_frame_id]
        success, pos, rot = self.get_transform_from_tf2(self.global_frame_id, self.base_frame_id)
        if success:
            p.resetBasePositionAndOrientation(base_link_sim_id, pos, rot)

    def update_robot_joints(self, joint_states_msg):
        self.update_robot_base()
        base_link_sim_id = self.bullet_node_id_map[self.base_frame_id]
        for joint_index in range(0, len(joint_states_msg.name)):
            join_sim_id = self.simulator_joint_id_map[joint_states_msg.name[joint_index]]
            p.resetJointState(base_link_sim_id, join_sim_id, joint_states_msg.position[joint_index], 0.0)

class PyBulletEnv(object):
    def __init__(self, robot_urdf=None, headless=True):
        if headless:
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(1)
        #p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        #p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
        #p.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
        #p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
        p.setAdditionalSearchPath('/home/twarz/catkin_ws/src/laas_objects/res/urdf')
        self.objects = {}
        self.object_names = {}

        if robot_urdf is not None:
            pass
            #self.robot_listener = RobotListener(robot_urdf)

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

    def compute_point_cloud(self, camera, max_range, width, height):
        return camera.get_depth_image(width, height, max_range)

    
    def compute_points_from_camera(self, camera, sample_size, max_range, sigma, sigma_pure, dt):
        #p.removeAllUserDebugItems()
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
            

        hits = p.rayTestBatch(ray_from, ray_to)
        
        point_clouds = {}

        element_center_attention = hits[0][0] # get object id
        center_point = hits[0][3] # get hit position
        max_radius = 1.0

        # name, pos, intensity
        selected_hits = self.compute_intensity(hits[0], hits[1:], max_radius, gamma=0.5, penalty=1.0, dt=dt)

        for h in selected_hits:
            #p.addUserDebugLine(camera.position, [h[1], h[2], h[3]], rayMissColor)
            if h[0] not in point_clouds.keys():
                point_clouds[h[0]] = []
            point_clouds[h[0]].append(h[1:])
        
        return point_clouds, element_center_attention, center_point

    def close(self):
        p.disconnect()

    def compute_intensity(self, hit_center, hits, max_radius, gamma, penalty, dt):
        ds = []
        d_clippeds = []
        d_norms = []
        #intensities = []
        selected_hits = []
        for h in hits:
            h_object = h[0]
            if h_object < 0:
                continue
            
            h_pos = np.array(h[3])
            d = math.sqrt(np.sum((hit_center[3] - h_pos)**2)) # Euler distance
            ds.append(d)

            d_clipped = min(d, max_radius)
            d_clippeds.append(d_clipped)
            d_norm = d_clipped / max_radius
            d_norms.append(d_norm)

            h_intensity = 1.0 - d_norm
            #intensities.append(h_intensity)
            h_intensity = h_intensity * dt
            h_intensity = h_intensity * penalty

            if h_object != hit_center[0]:
                h_intensity = h_intensity * gamma

            if h_intensity > 1e-4:
                selected_hits.append([h_object, h_pos[0], h_pos[1], h_pos[2], h_intensity])
        return selected_hits

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
    def __init__(self, human_attentions_topic, headless, robot_urdf=None):
        self.pc_pub = rospy.Publisher(human_attentions_topic, HumanAttentionArray, queue_size=1)
        self.pybullet_env = PyBulletEnv(robot_urdf=robot_urdf ,headless=headless)
        self.cameras = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pybullet_env.add_object('plane.urdf', 'plane', [0, 0, -2])
        #self.pybullet_env.add_object('table.urdf', 'table', [-1, 0, -1.5])
        self.update_camera('human_demo', [-2, 1, 0], [0, -0.5, 0, 1], 0.02)
        self.rate = rospy.Rate(20) # 30hz
        self.last_stamp = None

        # add PR2 robot
        '''
        self.pybullet_env.add_object('pr2_base.urdf', 'pr2_base', [-1, 0, -0.9])
        self.pybullet_env.add_object('pr2_body.urdf', 'pr2_body', [-1, 0, -0.45])
        self.pybullet_env.add_object('pr2_head.urdf', 'pr2_head', [-1, 0, 0.15])
        self.pybullet_env.add_object('pr2_arm.urdf', 'pr2_larm', [-0.65, 0.15, -0.30])
        self.pybullet_env.add_object('pr2_arm.urdf', 'pr2_rarm', [-0.65, -0.15,-0.30])
        self.pybullet_env.add_object('pr2_grip.urdf', 'pr2_lgrip', [-0.65, 0.15, -0.1])
        self.pybullet_env.add_object('pr2_grip.urdf', 'pr2_rgrip', [-0.65, -0.15,-0.1])
        '''

        # add Pepper robot
        '''
        self.pybullet_env.add_object('pepper_base.urdf', 'pepper_base', [-1, 1, -0.9])
        self.pybullet_env.add_object('pepper_body.urdf', 'pepper_body', [-1, 1, -0.55])
        self.pybullet_env.add_object('pepper_tablet.urdf', 'pepper_tablet', [-0.8, 1, -0.55])
        self.pybullet_env.add_object('pepper_head.urdf', 'pepper_head', [-1, 1, -0.2])
        self.pybullet_env.add_object('pepper_arm.urdf', 'pepper_larm', [-1, 1.2, -0.5])
        self.pybullet_env.add_object('pepper_arm.urdf', 'pepper_rarm', [-1, 0.8,-0.5])
        self.pybullet_env.add_object('pepper_hand.urdf', 'pepper_lhand', [-1, 1.2, -0.6])
        self.pybullet_env.add_object('pepper_hand.urdf', 'pepper_rhand', [-1, 0.8,-0.6])
        '''
        print("test")
        super(VisualAttentionUWDS, self).__init__("visual_attention", MONITOR)
        print("test")
        
    def update_camera(self, cam_id, position, orientation, dt):
        if cam_id in self.cameras.keys():
            self.cameras[cam_id].update(position, orientation, dt)
        else:
            self.cameras[cam_id] = PyBulletVirtualCamera(RESOLUTION, position=position, orientation=orientation)
        return self.cameras[cam_id]

    def remove_camera(self, node_id):
        if node_id in self.cameras.keys():
            del self.cameras[node_id]

    def publish_visual_attention(self, dt):
        # finally, trigger raycasting
        # p.stepSimulation()
        attentions = HumanAttentionArray()
        attentions.header.frame_id = 'robot/merged_stable/human_demo'
        attentions.header.stamp = self.last_header.stamp
        if self.cameras:
            sigma = 0.3
            for camera_id, camera in self.cameras.items():
                #camera_frame = world_name + '/' + camera_id
                #start = time.time()
                #sigma = 0.5 #min(max(camera.speed, 0.2), 0.8)
                #point_clouds, center_element, center_point = self.pybullet_env.compute_points_from_camera(camera, SAMPLE_SIZE, MAX_RANGE, sigma*RESOLUTION[0]/2., sigma, dt)
                #print(time.time() - start)
                point_clouds, center_element, center_point = self.pybullet_env.compute_point_cloud(camera, 10, 16, 16)

                human_attention = HumanAttention()
                human_attention.human_id = camera_id
                human_attention.element_of_attention = self.pybullet_env.get_name(center_element)
                human_attention.point_of_attention.x = center_point[0]
                human_attention.point_of_attention.y = center_point[1]
                human_attention.point_of_attention.z = center_point[2]

                for pc_id, pc in point_clouds.items():
                    #print(self.last_header.frame_id)
                    #pc_header = Header(stamp=self.last_header.stamp, frame_id=self.last_header.frame_id)
                    pc_header = Header(stamp=self.last_header.stamp, frame_id='robot/merged_stable/human_demo')
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

    def onChanges(self, world_name, header, invalidations):
        dt = 0.0
        if self.last_stamp is not None:
            dt = (header.stamp - self.last_stamp).to_sec()
        self.last_stamp = header.stamp

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
                self.pybullet_env.update_object('human_head.urdf', node_id, position, orientation)
                #self.pybullet_env.update_object('human_body.urdf', node_id+'_body', position)
                self.update_camera(node_id, position, orientation, dt)

        self.last_header = header

    def spin(self):
        last_stamp = None
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            dt = 0.0
            if last_stamp is not None:
                dt = (stamp - last_stamp).to_sec()
            last_stamp = stamp
            self.publish_visual_attention(dt)
            self.rate.sleep()
        self.pybullet_env.close()

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
    human_attentions_topic = '/humans/visual/current'
    headless = rospy.get_param("~headless", True)
    pr2_urdf = rospy.get_param("~robot_urdf", None)
    visual_attention = VisualAttentionUWDS(human_attentions_topic, headless, robot_urdf=pr2_urdf)
    print("ready to spin")
    visual_attention.spin()
''' 

if __name__ == '__main__':
    rospy.init_node("visual_attention") 
    gaze_topic_sub = "/humans/poses/3D/gaze"
    point_cloud_topic = '/humans/visual_attention'
    visual_attention = VisualAttention(gaze_topic_sub, point_cloud_topic)
    rospy.spin()
    visual_attention.pybullet_env.close()
'''