#!/usr/bin/env python3
import rospy
import ros_numpy
from std_msgs.msg import ColorRGBA, Int32
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Twist
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf.transformations
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import message_filters
import matplotlib.pyplot as plt
from ultralytics import YOLO
import numpy as np
import math
import cv2
import sys, select, termios, tty

def find_valid_elements(subarray):
    valid_elements = []
    for i in range(subarray.shape[0]):  # Iterate through channels
        channel = subarray[i, :, :]
        # Check if there are any non-nan and non-inf elements in this channel
        if np.any(~np.isnan(channel) & ~np.isinf(channel)):
            # Find the first valid element in this channel for simplicity
            valid_idx = np.where(~np.isnan(channel) & ~np.isinf(channel))
            first_valid = channel[valid_idx[0][0], valid_idx[1][0]]
            valid_elements.append(first_valid)
        else:
            # If no valid element is found, use a default value to indicate an invalid state
            print("No good")
            return None
    return valid_elements

def boundingbox(loc):
    [x,y,w,h] = loc
    x0 = int(x - w/2)
    y0 = int(y - h/2)
    x1 = int(x - w/2)
    y1 = int(y + h/2)
    x2 = int(x + w/2)
    y2 = int(y + h/2)
    x3 = int(x + w/2)
    y3 = int(y - h/2)
    # print((x0,y0), (x1,y1), (x2,y2), (x3,y3))
    return [(x0,y0), (x1,y1), (x2,y2), (x3,y3)]

def isRealNumber(number):
        return not np.isnan(number) and not np.isinf(number)
# input: x,y,z
def all_real(*args):
        """Check if all arguments are real numbers."""
        return all(isRealNumber(arg) for arg in args)
# input (x1,y1,z1), (x2,y2,z2)
def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def inRange(x,y):
    print("not in rage: ",0 <= x <= 5 and -1 <= y <= 1)
    return 0 <= x <= 5 and -1 <= y <= 1

def calculate_angle_and_distance(x, y, current_x, current_y):
    x_ = x - current_x
    y_ = y - current_y
    angleToGoal = math.atan2(y_, x_)
    distance = math.sqrt(y_ ** 2 + x_ ** 2)
    return angleToGoal, distance

def calculate_3d_angles(current_position, goal_position):
    """
    Calculate azimuth and elevation angles from current_position to goal_position.    :param current_position: Tuple or list with current (x, y, z) coordinates.
    :param goal_position: Tuple or list with goal (x, y, z) coordinates.
    :return: Tuple with azimuth and elevation angles in radians.
    """
    # Calculate direction vector from current_position to goal_position
    direction_vector = np.array(goal_position) - np.array(current_position)    # Normalize the direction vector
    direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)    # Calculate azimuth angle (angle in the XY plane)
    # azimuth = np.arctan2(direction_vector_normalized[1], direction_vector_normalized[0])    # Calculate elevation angle (angle from XY plane towards Z axis)
    elevation = np.arcsin(direction_vector_normalized[2])    
    return elevation

def shortest_angular_difference(angle1, angle2):
    """
    Calculate the shortest angular difference between two angles in degrees.
    The result is adjusted to be within the range [-180, 180] degrees.
    """
    # Difference between the two angles
    delta_angle = angle1 - angle2
    # Normalize the difference to be within -pi to pi
    error_rad = np.arctan2(np.sin(delta_angle), np.cos(delta_angle))
    return error_rad

def turning(current_theta, angleToGoal,angular_speed = 1):
    cmd_angular_z = 0  # This will hold the angular speed to return

    if angleToGoal >= 0 and current_theta >= 0:  # positive/on the right
        if angleToGoal > current_theta:
            cmd_angular_z = angular_speed
        else:
            cmd_angular_z = -angular_speed

    elif angleToGoal <= 0 and current_theta <= 0:  # negative/on the left
        if angleToGoal > current_theta:
            cmd_angular_z = angular_speed
        else:
            cmd_angular_z = -angular_speed

    else:  # one is positive and one is negative
        if angleToGoal < 0:  # angleToGoal is negative && current is positive
            bot = (angleToGoal - math.pi) + (math.pi - current_theta)
            top = (0 - angleToGoal) + current_theta
            if top > bot:
                cmd_angular_z = -angular_speed
            else:
                cmd_angular_z = angular_speed
        else:  # angleToGoal is + && current is -
            bot = (current_theta - math.pi) + (math.pi - angleToGoal)
            top = (0 - current_theta) + angleToGoal
            if top > bot:
                cmd_angular_z = angular_speed
            else:
                cmd_angular_z = -angular_speed

    return cmd_angular_z

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# index, class, center x, center y, width, height
class obs_def_:
    # def __init__(self, index, clss, p, loc):
    def __init__(self, index, clss, p, loc):
        self.index = index
        self.clss = clss
        [x,y,w,h] = loc
        self.x = x
        self.y = y
        self.w = w 
        self.h = h 
        self.x0 = int(x - w/2)
        self.y0 = int(y - h/2)
        self.x1 = int(x - w/2)
        self.y1 = int(y + h/2)
        self.x2 = int(x + w/2)
        self.y2 = int(y + h/2)
        self.x3 = int(x + w/2)
        self.y3 = int(y - h/2)
        self.area = h*w
        self.pose = (0,0,0)
        self.c0 = (0,0)
        self.c1 = (0,0)
        self.c2 = (0,0)
        self.c3 = (0,0)
        

        self.c0_pose = (0,0,0)
        self.c1_pose = (0,0,0)
        self.c2_pose = (0,0,0)
        self.c3_pose = (0,0,0)

        self.reach = None

        self.b = 1.0
        (H,W) = (540,960)      
    # pixel (x,y)
    def getCenter(self):
        return (self.x,self.y)        
    # set center pose
    def setPose(self, p):
        self.pose = p    
    # get center pose
    def getPose(self):
        return self.pose
    # set pixel coordinate of corners
    def setCorners(self,c0,c1,c2,c3):
        self.c0 = c0
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
    # 4 corners in pixel
    def getCorners(self):
        return (self.x0,self.y0),(self.x1,self.y1),(self.x2,self.y2),(self.x3,self.y3)  
    # set 4 pose in (x,y,z)
    def setCornersPose(self, c0,c1,c2,c3):
        self.c0_pose = c0
        self.c1_pose = c1
        self.c2_pose = c2
        self.c3_pose = c3
    # 4 corner pose
    def getCornersPose(self):
        return self.c0_pose, self.c1_pose, self.c2_pose, self.c3_pose
    # 1 = spall, 0 = crack
    def getClass(self):
        return self.clss
    # id to keep track
    def getIndex(self):
        return self.index
    # area of defect in pixels
    def getArea(self):
        return self.area
    # set reach (x,y,z)
    def setReach(self,r):
        self.reach = r  
    # get reach (x,y,z)
    def getReach(self):
        return self.reach
    # get reach (x,y)
    def getReach2D(self):
        return (self.reach[0], self.reach[1])
    # set b
    def setB(self,b):
        self.b = b
    # get b
    def getB(self):
        return self.b


class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0
        self.last_time = rospy.Time.now()

    def update(self, current_error):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_time).to_sec()
        if delta_time <= 0:
            # Prevent division by zero and ensure delta_time is positive
            print("DELTA_T IS NEGATIVE")
            return 0
        
        derivative = (current_error - self.previous_error) / delta_time
        output = self.kp * current_error + self.kd * derivative
        
        self.previous_error = current_error
        self.last_time = current_time
        
        return output
    
    def updateAng(self, current, goal):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_time).to_sec()
        if delta_time <= 0:
            # Prevent division by zero and ensure delta_time is positive
            print("DELTA_T IS NEGATIVE")
            return 0
        neg = -1
        current_error = shortest_angular_difference(current,goal)
        if shortest_angular_difference(current,goal) < 0:
            current_error = abs(current_error)
            neg = 1

        derivative = (current_error - self.previous_error) / delta_time
        output = self.kp * current_error + self.kd * derivative
        
        self.previous_error = current_error
        self.last_time = current_time
        
        return output * neg
    
class defect_detection:
    def __init__(self):
        self.bridge = CvBridge()
        self.cloud_map = None
        self.cloud_header = None
        self.rgb_map = None
        self.rgb_seg_map = None
        self.xyz_array = None
        self.vis_def_area = False
        self.vis_reach = True
        self.vis_b = False
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.yaw = 0
        self.cmd_vel = Twist()
        self.lin_tol = 0.06
        self.ang_tol = 0.1
        self.cur_def_ind = -1
        # 1 = turn, 2 = ER
        self.action = -1
        self.lin_control = PDController(0.1,0.0) # 0.25, 0
        self.ang_control = PDController(1.8,0.3)
        self.speed_ar = []
        self.frontier = np.array([])
        self.black_list = []
        self.def_index_black_list = []

        self.rate = rospy.Rate(10)  # 10hz
        self.odom_icp_sub = rospy.Subscriber("/zed/zed_nodelet/odom", Odometry, self.odomCallBack)
        self.frontier_sub = rospy.Subscriber("/explore/frontiers", MarkerArray, self.frontierCallBack)   
        # self.range1 = rospy.Subscriber("/range1", Int32, self.range1Cb)     
        # self.range2 = rospy.Subscriber("/range2", Int32, self.range2Cb)
        # self.range2 = rospy.Subscriber("/imu", Int32, self.armIMUCb)          
        # time synch
        self.rgb_sub = message_filters.Subscriber("/zed/zed_nodelet/rgb/image_rect_color", Image)
        self.cloud_sub = message_filters.Subscriber("/zed/zed_nodelet/point_cloud/cloud_registered", PointCloud2)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.cloud_sub], 10)
        self.ts.registerCallback(self.callback)

        self.vis_pub = rospy.Publisher("potential_defect", MarkerArray, queue_size=1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.target_pub = rospy.Publisher("target", Marker, queue_size=1)
        # self.arm_pub_ = rospy.Publisher('J1', Int32, queue_size=5)
        # self.lin_pub_ = rospy.Publisher('J2', Int32, queue_size=5)
        self.border_ar = MarkerArray()
        self.reach_ar = MarkerArray()
        self.text_ar = MarkerArray()
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.mainCb)
        self.list_def_state = []
        self.z_constraint = 0.86
        self.y_contraint = 3.8
        # ER distance
        self.range1_dist = 100
        self.range2_dist = 100
        # arm angle
        self.arm_angle = None
        self.begin = rospy.Time.now()
        
        # YOLO
        model_path = "/home/ara/catkin_ws/src/detection/model/m_cul1+2.pt" 
        print('load model.......')
        self.model = YOLO(model_path)
        print('model loaded')
        # index stuff
        self.last_markers_count = 0
        self.index = 0
        # tf stuff
        self.tf_buffer = tf2_ros.Buffer()
        self.t1= tf2_ros.TransformListener(self.tf_buffer)

        self.start = False
        # wait for message before doing anything
        rospy.wait_for_message("/zed/zed_nodelet/rgb/image_rect_color", Image)
        rospy.wait_for_message("/zed/zed_nodelet/point_cloud/cloud_registered", PointCloud2)
        rospy.wait_for_message("/zed/zed_nodelet/odom", Odometry)
        rospy.wait_for_message("/explore/frontiers", MarkerArray)

        
        self.start = True
        rospy.on_shutdown(self.publish_velocity)

    def transform_point(self,x,y,z, child_frame, parent_frame):
        p = Point()
        p.x = x 
        p.y = y
        p.z = z                
        trans = self.tf_buffer.lookup_transform(parent_frame, child_frame,
                            rospy.get_rostime() - rospy.Duration(0.2),
                            rospy.Duration(0.1))
        # for bag
        # trans = self.tf_buffer.lookup_transform("odom", "zed_left_camera_frame",
        #                     header.stamp,
        #                     rospy.Duration(0.1))

        point_stamp = PointStamped()
        point_stamp.point = p
        point_stamp.header = "map"
        p_tf = do_transform_point(point_stamp, trans)

        return p_tf.point.x, p_tf.point.y, p_tf.point.z

    def frontierCallBack(self, msg):
        self.frontier = np.array([[marker.pose.position.x, marker.pose.position.y] for marker in msg.markers if marker.type == Marker.SPHERE])

        # Filter frontier by removing blacklist points
        self.filter_blacklist()

    def filter_blacklist(self, threshold = 0.2):
        if not self.frontier.size or not len(self.black_list):
            return
        
        # Calculate pairwise squared distances using broadcasting and vectorization
        # This avoids computing the square root for efficiency, comparing against the squared threshold instead
        diff = self.frontier[:, np.newaxis, :] - np.array(self.black_list)
        squared_distances = np.einsum('ijk,ijk->ij', diff, diff)
        min_squared_distances = squared_distances.min(axis=1)
    
        # Filter out frontier points within the threshold distance of any blacklist point
        # Note: threshold squared is compared to avoid sqrt for performance
        self.frontier = self.frontier[min_squared_distances > threshold**2]

    
        """this is for matching, but threshhold is better"""
        # # Large scale = python set
        # frontier_set = set(map(tuple, self.frontier))
        # blacklist_set = set(map(tuple, self.blacklist))        
        # # Filter out blacklist points
        # filtered_frontier = np.array(list(frontier_set - blacklist_set))
        # # Update frontier with filtered points
        # self.frontier = filtered_frontier

        # # small scale = broadcasting to find matches between frontier and blacklist
        # matches = (self.frontier[:, None] == self.blacklist).all(-1).any(-1)
        # # Filter out matching rows
        # self.frontier = self.frontier[~matches]

    def callback(self, image, cloud):
        try:
            self.rgb_map = self.bridge.imgmsg_to_cv2(image, "bgr8")
            # print("RGB shape: ",self.rgb_map.shape)
            self.rgb_seg_header = image.header
        except CvBridgeError as e:
            print(e)
            return
        
        self.xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud)
        self.cloud_map = cloud

        # get detection
        results = self.model(source=self.rgb_map, verbose=False)
        #
        for box in results[0]:   
            skip_to_next_box = False              
            loc = box.boxes.xywh.cpu()
            loc = loc.numpy()
            if box.boxes.conf.item() >= 0.5:
                # get corners [c0,c1,c2,c3]
                cs = boundingbox(loc.flatten().astype('int32'))
                corner_pose_list = []
                if self.vis_def_area:
                # for each corner
                    for c in cs:
                        x_val, y_val, z_val = self.xyz_array[c[1]][c[0]-2]
                        # print(x_val, y_val, z_val)
                        # check corners pose is real number
                        if all_real(x_val, y_val, z_val):
                            x,y,z = self.transform_point(x_val, y_val, z_val, "zed_left_camera_frame", "map")
                            corner_pose_list.append((x,y,z))
                        else:
                            # skip if any is nan
                            rospy.logwarn("corner not real")
                            skip_to_next_box = True
                            break
                # print(corner_pose_list)
                # get center pose
                r = loc.flatten().astype('int32')[1]
                c = loc.flatten().astype('int32')[0]
                x_val, y_val, z_val = self.xyz_array[r,c]
                # print(self.xyz_array[0,0].shape)
                # xyz_array = np.squeeze(self.xyz_array)
                # print(xyz_array.shape)
                # center pose or corner pose is not real number, skip to next
                # print(in)
                # if not all_real(x_val, y_val, z_val) or skip_to_next_box or not inRange(x_val, y_val):
                if not all_real(x_val, y_val, z_val):
                    rospy.logwarn("pose not real (%.2f,%.2f,%.2f), looks surrounding", x_val, y_val, z_val)
                    continue
                    xyz_array = np.squeeze(self.xyz_array)
                    # print(xyz_array.shape)
                    search_range = 10# Create a mask for valid and invalid pixels
                    valid_mask = ~np.isnan(xyz_array).any(axis=-1) & ~np.isinf(xyz_array).any(axis=-1)# Handling the edge cases by padding the valid_mask
                    padded_valid_mask = np.pad(valid_mask, pad_width=search_range, mode='constant', constant_values=False)# Finding a valid pixel:
                    # Instead of looking for the first valid pixel in spatial order,
                    # we identify any valid pixel within the range for demonstration purposes
                    # due to the complexity of replicating the exact loop behavior vectorized
                    r_padded, c_padded = r + search_range, c + search_range  # Adjust for padding offset# Search in the padded_valid_mask for valid pixels
                    found_valid = np.argwhere(padded_valid_mask[r_padded-search_range:r_padded+search_range+1,
                                                                c_padded-search_range:c_padded+search_range+1])
                    if found_valid.size > 0:
                        # Assuming the first found valid pixel is the one we want (for simplicity)
                        first_valid_relative = found_valid[0]
                        first_valid = (first_valid_relative[0] + r - search_range,
                                    first_valid_relative[1] + c - search_range)
                        x_val, y_val, z_val = xyz_array[first_valid[0], first_valid[1], :]
                        rospy.logwarn("found valid: %.2f, %.2f, %.2f",x_val, y_val, z_val)
                    else:
                        rospy.logwarn("No valid neighbor found within the specified range.")
                        continue
                # if not then transform to the map frame
                x, y, z = self.transform_point(x_val, y_val, z_val, "zed_left_camera_frame", "map")
                # print(x_val, y_val, z_val,x, y, z)
                # if too high, meaning out of reach
                if z > self.z_constraint or x > 3.5 or x < 1.4:
                    rospy.logwarn("out of range")
                    continue
                # debug
                color = (0,0,0)
                # print(box.boxes.cls.item())
                if box.boxes.cls.item() == 1.0:
                    color = (0,255,0)
                elif box.boxes.cls.item() == 0.0:
                    color = (0,0,255)
                # print("class ", box.boxes.cls.item())
                cv2.rectangle(self.rgb_map, cs[0], cs[2], color=color, thickness=2)
                # check if it is near
                for i in self.list_def_state:
                    # if near and same class
                    distance = euclidean_distance(i.getPose(), (x,y,z))
                    # rospy.logwarn("distance: %.2f, class: %.1f, %.1f",distance, box.boxes.cls.item(),i.getClass())
                    if distance < 1 and int(box.boxes.cls.item()) == int(i.getClass()):
                        # break then skip
                        _,_, w, h = loc.flatten().astype('int32')
                        i.setB(  i.getB() * ( (w*h)/(540*960) )    )
                        skip_to_next_box = True
                        break                
                if skip_to_next_box:
                    continue    
                # if meet all condition, then save it
                defect = obs_def_(self.index, box.boxes.cls.item(), box.boxes.conf.item(), loc.flatten().astype('int32'))
                defect.setPose((x,y,z))
                if self.vis_def_area:
                    defect.setCornersPose(corner_pose_list[0],corner_pose_list[1],corner_pose_list[2],corner_pose_list[3])
                _,_, w, h = loc.flatten().astype('int32')
                defect.setB(  defect.getB() * ( (w*h)/(540*960) )    )
                if self.robot_y > y:
                    # due to the set up of culvert, 0 can never = y since y is wall and robot can't have same position as
                    # wall unless it smash through it
                    reach = (x,y+(0.83*0.95),0) 
                else:
                    reach = (x,y-(0.83*0.8),0) 
                defect.setReach(reach)
                # put in                 
                self.list_def_state.append(defect)
                self.index+=1
        # print(len(self.def_index_black_list))
        # print(len(self.list_def_state))
        # rospy.logwarn("index: %d", self.index)
        # visualize in rviz
        # for de in self.list_def_state:
        #     self.visualizeDefect(de)
        # self.vis_pub.publish(self.border_ar)
        # self.vis_pub.publish(self.reach_ar)
        cv2.imshow("Image window", self.rgb_map)
        cv2.waitKey(3)

    def odomCallBack(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)        
        # Update yaw
        self.yaw = yaw

    def mainCb(self,event=None):        
        if not self.start:
            rospy.logwarn("Not yet")
            return
        return
        if (rospy.Time.now() - self.begin).to_sec() < 1:
            rospy.logwarn_once("waiting: %d", (rospy.Time.now() - self.begin).to_sec())
            return
        # print(getKey())
        # self.def_index_black_list.append(0)
        # print("blacklist:", self.def_index_black_list)
        # return
        # test goTo
        # if self.action != 1 and self.goTo(0.3,0.3):
        #     self.action = 1
        # return
        # return
        if len(self.def_index_black_list) > 3: 
            rospy.logwarn("OVER 2")
            return

        # when u reach turn
        # if self.action == 1:
        #     defect = self.list_def_state[self.cur_def_ind].getPose()
        #     if self.goTo(defect[0], defect[1]):
        #         # self.action = 2
        #         # TODO this is suppose to be at the end
        #         self.action = -1
        #         self.cur_def_ind = -1
        #         """TODO: create a fake complete action to test action but still using manual arm"""
        #     return
        
        # # check if high B
        # if self.action == 2:
        #     beginning = rospy.Time.now()
        #     while True:
        #         # pause for 4 sec, if belief go up enough, then proceed to next one, if not go next
        #         if self.list_def_state[self.cur_def_ind].getB() > 0.01:
        #             self.action = 3
        #             break
        #         else:
        #             if (rospy.Time.now() - beginning).to_sec() > 4:
        #                 self.cur_def_ind = -1
        #                 self.action = -1
        #                 break
        #     return
        # ER stuff angle
        # if self.action == 3:
        #     angleToGoal3D = calculate_3d_angles((self.robot_x, self.robot_y, self.robot_z),self.list_def_state[self.cur_def_ind].getPose()) * math.pi/180
        #     arm_ang = Int32()
        #     arm_ang.data = 0
        #     while True:
        #         # angle too low
        #         if self.arm_angle < angleToGoal3D - 10:  
        #             arm_ang.data = 4
        #             # angle too high
        #         elif self.arm_angle > angleToGoal3D + 10:  
        #             arm_ang.data = 3
        #         else:
        #             self.arm_pub_.publish(arm_ang)
        #             self.action = 4
        #             break
        #         self.arm_pub_.publish(arm_ang)
        # # ER stuff out
        # if self.action == 4:
        #     lin_act = Int32()
        #     while True:
        #         if self.range1_dist < 5 or self.range2_dist < 5:
        #             self.action = 5
        #             break
        #         else:
        #             lin_act.data = 1
        #             self.lin_pub_.publish(lin_act)
        # # ER stuff in
        # if self.action == 5:
        #     lin_act = Int32()
        #     lin_act.data = 2
        #     beginning = rospy.Time.now()
        #     while True:
        #         # wait for 2 sec for ER to measure
        #         if (rospy.Time.now() - beginning).to_sec() > 2:
        #             beginning = rospy.Time.now()
        #             # lin act back in for 5 sec
        #             while (rospy.Time.now() - beginning).to_sec() < 5:
        #                 self.lin_pub_.publish(lin_act)
        #             # reset and break
        #             self.action = 6
        #             break
        
        # # ER angle back to 45 
        # if self.action == 6:
        #     arm_ang = Int32()
        #     arm_ang.data = 0
        #     while True:
        #         # angle too low
        #         if self.arm_angle < 40:  
        #             arm_ang.data = 4
        #             # angle too high
        #         elif self.arm_angle > 50:  
        #             arm_ang.data = 3
        #         else:
        #             self.arm_pub_.publish(arm_ang)
        #             self.action = -1
        #             break
        #         self.arm_pub_.publish(arm_ang)
        
        # rospy.logwarn("test")
        # self.goTo(0.3,0.3)
        if self.action == 1:
            # while True:
            #     if getKey() == '0':
            #         self.action = -1
            #         break
            # return
            if getKey() == '0':
                self.action = -1     
                rospy.logwarn("done, I'm out    ") 
            else:
                rospy.logwarn("manual")
            return
        if getKey() == '9':
            while True:
                self.publish_velocity(0,0)
                if getKey() == '0':
                    rospy.logwarn("out    ") 
                    break
                else:
                    rospy.logerr("stop    ") 
        
        """TODO: test to make sure goTo is good"""
        # if there's a potential defect
        if len(self.list_def_state) > 0 and len(self.list_def_state) > len(self.def_index_black_list):
            # rospy.logwarn("go")
            # RVIZ
            if self.vis_def_area or self.vis_reach:
                for de in self.list_def_state:
                    self.visualizeDefect(de)
            if self.vis_def_area:
                self.vis_pub.publish(self.border_ar)
            if self.vis_reach:
                self.vis_pub.publish(self.reach_ar)

            # if there's only 1 then min_index = 0 
            closest_index = 0
            # else do more calculation
            if len(self.list_def_state) > 1:
                poses = np.array([defect.getReach2D() for defect in self.list_def_state])
                # Create an array of indices to keep, excluding blacklist index
                all_indices = np.arange(len(self.list_def_state))
                keep_indices = np.setdiff1d(all_indices, np.array(self.def_index_black_list))
                # Filter poses to only those not in blacklist_indices
                filtered_poses = poses[keep_indices]
                # Compute squared Euclidean distances (no need to take the square root for comparison)
                distances_squared = np.sum((filtered_poses - np.array([self.robot_x, self.robot_y]))**2, axis=1)
                # Find the index of the closest obs_def_ in the filtered list
                min_index = np.argmin(distances_squared)
                closest_index = keep_indices[min_index]  # Map back to original list index
                self.cur_def_ind = closest_index
                # print("all:", all_indices)
                # print("keep",keep_indices)
                # print("blacklist:",self.def_index_black_list)
            # get goal
            (goal_x, goal_y) = self.list_def_state[closest_index].getReach2D()
            if (self.goTo(goal_x, goal_y)):
                # if reach, next action which is turning
                self.action = 1
                """TODO: 1 create a fake complete action to test decision using manual
                         comment out the self.publish_velocity()
                         blacklist defect index
                """
                rospy.logwarn("REACH GOAL %d", closest_index)
                # add to defect black list to 
                rospy.logwarn("appending index %d", closest_index)
                self.def_index_black_list.append(closest_index)
                # print("blacklist:", self.def_index_black_list)
                return
            else:
                # rospy.logwarn("current index %d", closest_index)
                pass
        # if no defect, then frontier
        else:
            rospy.logwarn("frontier")
            # go to the frontier already sorted by weight
            if self.goTo(self.frontier[0,0], self.frontier[0,1]):
                # blacklist the frontier when reach                
                self.black_list.append(self.frontier[0])


    def goTo(self, x, y):
        # TODO: Test
        # self.visualizeTarget(x,y)
        # return
        self.visualizeTarget(x,y)
        lin_vel = 0
        ang_vel =0  
        angleToGoal, distance = calculate_angle_and_distance(x,y,self.robot_x, self.robot_y)
        """ if close enough """
        if distance < 0.15:
            self.publish_velocity(lin_vel, ang_vel)
            return True                
        
        angle_diff = abs(shortest_angular_difference(self.yaw, angleToGoal)) 
        # self.speed_ar.append(ang_vel)
        # self.err_ar.append(angle_diff)
        """only go forward if angle difference is low enough"""
            # print("speed=",self.speed_ar)
            # print("err=", self.err_ar)
            

        if angle_diff >= 0.25:
            ang_vel = round(self.ang_control.updateAng(self.yaw, angleToGoal),5) #* turning(self.yaw, angleToGoal)
            rospy.logwarn("yaw: %.2f, atg: %.2f, angl diff: %.2f, ang_vel: %.2f", np.degrees(self.yaw), np.degrees(angleToGoal), np.degrees(angle_diff), ang_vel)
            lin_vel = 0
        if angle_diff < 0.25:
            ang_vel =0
            lin_vel = self.lin_control.update(distance)
            rospy.logwarn("lin_vel: %.2f", lin_vel)

        self.publish_velocity(lin_vel, ang_vel)
        return False


        # rospy.logwarn("angleToGoal: %.2f", angleToGoal)
        ang_vel = 0
        lin_vel = 0
        # if action is 1, then only turn
        if self.action == 1:
            rospy.logerr("atg: %.2f", angleToGoal)
            self.visualizeTarget(x,y)
            if abs(angleToGoal - self.yaw) > self.ang_tol:
                ang_vel = turning(self.yaw, angleToGoal)
                self.publish_velocity(0, ang_vel)
                return False
            else:
                return True

        # turn if angle diff above lin_tol
        if abs(angleToGoal - self.yaw) > self.ang_tol:
            ang_vel = turning(self.yaw, angleToGoal)
        elif abs(angleToGoal - self.yaw) < 0.3:
            ang_vel = turning(self.yaw, angleToGoal, 0.1)
            # rospy.logwarn("ang_vel: %.2f", ang_vel)
        # forward if distance above ang_tol, combine with turn if angle diff close to ang_tol
        if distance > self.lin_tol and abs(angleToGoal - self.yaw) < 0.3 and abs(angleToGoal - self.yaw) > self.ang_tol:
            lin_vel = 0.1
        
        # check if goal reached
        if distance <= self.lin_tol:
            self.publish_velocity(0, 0)
            rospy.logwarn("Goal Reached!",)
            return True
        else:  
            rospy.logerr("distance: %.2f, atg: %.2f", distance, angleToGoal)
            self.publish_velocity(lin_vel, ang_vel)     
            self.visualizeTarget(x,y)  
            return False

    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publishes velocity commands to cmd_vel."""
        self.cmd_vel.linear.x = linear_x
        self.cmd_vel.angular.z = angular_z
        self.cmd_pub.publish(self.cmd_vel)

    def range1Cb(self, msg):
        self.range1_dist = msg.data

    def range2Cb(self, msg):
        self.range2_dist = msg.data
    
    def armIMUCb(self, msg):
        self.arm_angle = msg

    def visualizeTarget(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust according to your TF frames
        # marker.header.stamp = rospy.Time.now()
        marker.ns = "arrow_marker"
        marker.id = 500
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set the pose of the marker
        # The arrow points up from the point (x, y) to (x, y, 1)
        marker.points.append(Point(x, y, 0))  # Start point
        marker.points.append(Point(x, y, 0.5))  # End point - pointing straight up
        
        # Set the scale of the arrow
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1  # Head diameter
        marker.scale.z = 0.1  # Head length
        
        # Set the color of the marker
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Make sure to set the alpha to something non-zero!
        
        # Publish the Marker
        self.target_pub.publish(marker)

    def visualizeDefect(self,obs_def_instance, frame_id="map"):
        color = ColorRGBA()
        color.a = 1.0
        if obs_def_instance.getClass() == 1.0:
            color.g = 1.0 
        elif obs_def_instance.getClass() == 0.0:
            color.r = 1.0

        if self.vis_def_area:
            # bbox in rviz with line strips
            line_strip = Marker()
            line_strip.header.frame_id= frame_id
            line_strip.type = Marker.LINE_STRIP
            line_strip.action = Marker.ADD
            line_strip.pose.orientation.w = 1.0
            line_strip.id = obs_def_instance.getIndex()
            line_strip.scale.x = 0.01
            line_strip.color = color
            # 4 lines
            for c in obs_def_instance.getCornersPose():
                p = Point()
                p.x = c[0] 
                p.y = c[1]                 
                p.z = c[2] 
                if obs_def_instance.getIndex() == 0:
                    p.y += 0.1
                line_strip.points.append(p)
            # add a 5th point to complete 4 lines into box 
            line_strip.points.append(line_strip.points[0])
            self.border_ar.markers.append(line_strip)

        if self.vis_reach:
            # reach pose
            reach_pose = Marker()
            reach_pose.header.frame_id= frame_id
            reach_pose.type = Marker.POINTS
            reach_pose.action = Marker.ADD
            reach_pose.pose.orientation.w = 1.0
            reach_pose.id = obs_def_instance.getIndex() + 200
            reach_pose.scale.x = 0.05
            reach_pose.scale.y = 0.05
            reach_pose.color = color
            p = Point()
            p.x = obs_def_instance.getReach()[0]
            p.y = obs_def_instance.getReach()[1] 
            p.z = obs_def_instance.getReach()[2]  
            reach_pose.points.append(p)
            self.reach_ar.markers.append(reach_pose)

        if self.vis_b:
            text_marker = Marker()
            text_marker.header.frame_id= frame_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obs_def_instance.getReach()[0]
            text_marker.pose.position.y = obs_def_instance.getReach()[1] 
            text_marker.pose.position.z = obs_def_instance.getReach()[2]  
            text_marker.color = color
            text_marker.scale.x = 0.1
            self.text_ar.append(text_marker)

if __name__ == '__main__':
    print('START')
    rospy.init_node('detection')
    print('Initialize Node')
    x=defect_detection()
    rospy.spin()
