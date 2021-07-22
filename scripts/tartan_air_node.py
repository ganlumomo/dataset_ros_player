# ros
import rospy
import tf
import tf2_sensor_msgs
import cv2
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tf2_ros import TransformStamped

# others
import numpy as np
from scipy.spatial.transform import Rotation as Rotation

def seg2vis(segnp):
    colors = [(205, 92, 92), (0, 255, 0), (199, 21, 133), (32, 178, 170), (233, 150, 122), (0, 0, 255), (128, 0, 0), (255, 0, 0), (255, 0, 255), (176, 196, 222), (139, 0, 139), (102, 205, 170), (128, 0, 128), (0, 255, 255), (0, 255, 255), (127, 255, 212), (222, 184, 135), (128, 128, 0), (255, 99, 71), (0, 128, 0), (218, 165, 32), (100, 149, 237), (30, 144, 255), (255, 0, 255), (112, 128, 144), (72, 61, 139), (165, 42, 42), (0, 128, 128), (255, 255, 0), (255, 182, 193), (107, 142, 35), (0, 0, 128), (135, 206, 235), (128, 0, 0), (0, 0, 255), (160, 82, 45), (0, 128, 128), (128, 128, 0), (25, 25, 112), (255, 215, 0), (154, 205, 50), (205, 133, 63), (255, 140, 0), (220, 20, 60), (255, 20, 147), (95, 158, 160), (138, 43, 226), (127, 255, 0), (123, 104, 238), (255, 160, 122), (92, 205, 92),]
    #segvis = np.zeros(segnp.shape+(3,), dtype=np.uint8)
    segvis = np.zeros(segnp.shape, dtype=np.uint8)
    for k in range(256):
        mask = segnp==k
        colorind = k % len(colors)
        if np.sum(mask)>0:
            #segvis[mask,:] = colors[colorind]
            segvis[mask] = colorind
    return segvis

class TartanAirData:
    def __init__(self):
        
        # define camera parameters
        self.fx = 320.0 # focal length x
        self.fy = 320.0 # focal length y
        self.cx = 320.0 # optical center x
        self.cy = 240.0 # optical center y
        self.width = 640
        self.height = 480

        self.init_trans_to_ground = np.array([[1, 0, 0, 0],
                                              [0, 0, 1, 0],
                                              [0, -1, 0, 1],
                                              [0, 0, 0, 1]], dtype = np.float32)

        self.br = CvBridge()

        # define msg publishers
        self.pc2_publisher = rospy.Publisher("points", PointCloud2, queue_size = 1)
        #self.pc2_global_publisher = rospy.Publisher("points_global", PointCloud2, queue_size = 1)
        self.pose_publisher = rospy.Publisher("pose", PoseWithCovarianceStamped, queue_size = 1)
        self.path_publisher = rospy.Publisher("path", Path, queue_size = 1)
        self.left_color_image_publisher = rospy.Publisher("left_color_image", Image, queue_size = 1)
        self.left_depth_image_publisher = rospy.Publisher("left_depth_image", Image, queue_size = 1)

        # run node
        rospy.init_node('tartan_air_node', anonymous = True)
        #rospy.Rate(30)
        seq_dir ="/media/ganlu/Samsung_T5/000_tro2020/tartanair-release1/abandonedfactory/Easy/P002/"
        left_camera_pose_file = seq_dir + "pose_left.txt"
        self.read_left_camera_poses(left_camera_pose_file)
        self.depth_left_dir = seq_dir + "depth_left/"
        self.image_left_dir = seq_dir + "image_left/"
        self.seg_left_dir = seq_dir + "seg_left/"
        
        # init path
        self.path = Path()
        self.path.header.frame_id = "map"
        self.process_scans(3717)


    def process_scans(self, scan_num):
        for scan_id in range(scan_num):
            rospy.sleep(3.5)

            stamp = rospy.Time.now()

            # load seg img
            #seg_left_name = self.seg_left_dir + "%06i" % scan_id + "_left_seg.npy"
            #seg_left = np.load(seg_left_name)
            #seg_left_vis = seg2vis(seg_left)
            #filename = seg_left_name[:len(seg_left_name)-8] + ".png"
            #print(filename)
            #cv2.imwrite(filename, seg_left_vis)

            # load left img
            image_left_name = self.image_left_dir + "%06i" % scan_id + "_left.png"
            print(image_left_name)
            image_left = cv2.imread(image_left_name)
            #print(image_left)
            self.left_color_image_publisher.publish(self.br.cv2_to_imgmsg(image_left))

            # load depth img
            depth_left_name = self.depth_left_dir + "%06i" % scan_id + "_left_depth.npy"
            depth_left = np.load(depth_left_name)
            self.left_depth_image_publisher.publish(self.br.cv2_to_imgmsg(depth_left))
            #print(depth_left)
            pc = self.depth_to_pc(depth_left)
            #print(pc)
            #print(pc.shape)
            
            # publish points in left camera
            pc2 = self.pc_to_pc2(pc, stamp)
            #print(pc2)

            # publish points in map
            '''transform = TransformStamped()
            #rotation_inv = Rotation.from_quat([self.left_camera_poses[scan_id][3], self.left_camera_poses[scan_id][4], self.left_camera_poses[scan_id][5], self.left_camera_poses[scan_id][6]]  ).inv()
            #q_inv = rotation_inv.as_quat()
            #trans = np.array([self.left_camera_poses[scan_id][i] for i in range(3)])
            #print(rotation_inv.as_dcm())
            #trans_inv = - np.dot(np.array(rotation_inv.as_dcm()) , trans)
            #print(trans_inv)
            transform.transform.translation.x = self.left_camera_poses[scan_id][0]
            transform.transform.translation.y = self.left_camera_poses[scan_id][1]
            transform.transform.translation.z = self.left_camera_poses[scan_id][2]
            transform.transform.rotation.x = self.left_camera_poses[scan_id][3]
            transform.transform.rotation.y = self.left_camera_poses[scan_id][4]
            transform.transform.rotation.z = self.left_camera_poses[scan_id][5]
            transform.transform.rotation.w = self.left_camera_poses[scan_id][6]
            pc2_global = tf2_sensor_msgs.do_transform_cloud(pc2, transform)
            pc2_global.header.frame_id = "map"
            print(self.left_camera_poses[scan_id][0], self.left_camera_poses[scan_id][1], self.left_camera_poses[scan_id][2],
                    self.left_camera_poses[scan_id][3], self.left_camera_poses[scan_id][4], self.left_camera_poses[scan_id][5],
                    self.left_camera_poses[scan_id][6])'''

            # publish tf
            br = tf.TransformBroadcaster()
            br.sendTransform((self.left_camera_poses[scan_id][0],
                self.left_camera_poses[scan_id][1],
                self.left_camera_poses[scan_id][2]),
                ((self.left_camera_poses[scan_id][3],
                self.left_camera_poses[scan_id][4],
                self.left_camera_poses[scan_id][5],
                self.left_camera_poses[scan_id][6])),
                stamp,
                "left_camera",
                "map")

            # publish poses
            pose = self.pose_with_covariance_stamped(scan_id, stamp)
            self.pose_publisher.publish(pose)

            # publish path
            self.add_pose_to_path(scan_id, stamp)
            self.path_publisher.publish(self.path)

            # publish msgs
            self.pc2_publisher.publish(pc2)
            #self.pc2_global_publisher.publish(pc2_global)

    def depth_to_pc(self, depth):
        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.

        """
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        valid = (depth > 0) & (depth < 255)
        #z = np.where(valid, depth / 256.0, np.nan)
        #x = np.where(valid, z * (c - cx) / fx, 0)
        #y = np.where(valid, z * (r - cy) / fy, 0)
        z = np.where(valid, depth, np.nan)
        x = np.where(valid, z * (c - self.cx) / self.fx, 0)
        y = np.where(valid, z * (r - self.cy) / self.fy, 0)
        return np.float32(np.dstack((x, y, z)))

    def pc_to_pc2(self, pc, stamp):
        '''Converts a numpy array to a sensor_msgs.msg.PointCloud2'''
        pc2 = PointCloud2()
        pc2.header.frame_id = "left_camera"
        pc2.header.stamp = stamp
        pc2.height = pc.shape[0]
        pc2.width = pc.shape[1]
        pc2.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
        pc2.is_bigendian = False # assumption
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc.shape[1]
        pc2.is_dense = True
        pc2.data = pc.tostring()
        return pc2

    def read_left_camera_poses(self, left_camera_poses_file):
        left_camera_poses = np.loadtxt(left_camera_poses_file)
        self.left_camera_poses = self.ned2cam(left_camera_poses)
        #print(self.left_camera_poses)
    
    def ned2cam(self, traj):
        '''
        transfer a ned traj to camera frame traj
        '''
        T = np.array([[0,1,0,0],
                      [0,0,1,0],
                      [1,0,0,0],
                      [0,0,0,1]], dtype=np.float32)
        T_inv = np.linalg.inv(T)
        new_traj = []
        
        #rotation_inv = Rotation.from_quat([self.left_camera_poses[scan_id][3], self.left_camera_poses[scan_id][4], self.left_camera_poses[scan_id][5], self.left_camera_poses[scan_id][6]]  ).inv()
        #q_inv = rotation_inv.as_quat()
        #trans = np.array([self.left_camera_poses[scan_id][i] for i in range(3)])
        #print(rotation_inv.as_dcm())
        #trans_inv = - np.dot(np.array(rotation_inv.as_dcm()) , trans)
        #print(trans_inv)
 
        for tt in traj:
            Rot = Rotation.from_quat([tt[3], tt[4], tt[5], tt[6]])
            #print(Rot.as_dcm())
            rot = Rot.as_dcm()
            ttt = np.array([[rot[0][0], rot[0][1], rot[0][2], tt[0]],
                [rot[1][0], rot[1][1], rot[1][2], tt[1]],
                [rot[2][0], rot[2][1], rot[2][2], tt[2]],
                [0, 0, 0, 1]], dtype=np.float32)
            tttt = self.init_trans_to_ground.dot(T).dot(ttt).dot(T_inv)
            Rot2 = Rotation.from_dcm([[tttt[0][0], tttt[0][1], tttt[0][2]],
                    [tttt[1][0], tttt[1][1], tttt[1][2]],
                    [tttt[2][0], tttt[2][1], tttt[2][2]]])
            quad = Rot2.as_quat()
            #print(quad)
            ttt_quad = np.array([tttt[0][3], tttt[1][3], tttt[2][3], quad[0], quad[1], quad[2], quad[3]])
            new_traj.append(ttt_quad)
        return np.array(new_traj)

    def pose(self, scan_id):
        pose = Pose()
        pose.position.x = self.left_camera_poses[scan_id][0]
        pose.position.y = self.left_camera_poses[scan_id][1]
        pose.position.z = self.left_camera_poses[scan_id][2]
        pose.orientation.x = self.left_camera_poses[scan_id][3]
        pose.orientation.y = self.left_camera_poses[scan_id][4]
        pose.orientation.z = self.left_camera_poses[scan_id][5]
        pose.orientation.w = self.left_camera_poses[scan_id][6]
        return pose

    def pose_with_covariance_stamped(self, scan_id, stamp):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        # set pose
        pose.pose.pose = self.pose(scan_id)
        # set covariance
        for i in range(36):
            pose.pose.covariance[i] = 0.0
        return pose

    def add_pose_to_path(self, scan_id, stamp):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose = self.pose(scan_id)
        self.path.poses.append(pose)
        
    def main(self):
        print("spin..")
        rospy.spin()
        

if __name__ == "__main__":
    tartan_air_data = TartanAirData()
    tartan_air_data.main()
