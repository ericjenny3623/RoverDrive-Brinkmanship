#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import PolygonStamped, Point, Point32, Pose
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData
import time

import tf2_ros
import numpy as np
import ros_numpy
import pcl  # Use this one for tx2: https://github.com/PonyPC/python-pcl-jetson
from alphashape import alphashape
from shapely.geometry import Polygon, LineString
from shapely.ops import cascaded_union
from pyquaternion import Quaternion

cloud_pub = rospy.Publisher('/brink/out/cloud', PointCloud2, queue_size=10)
poly_pub = rospy.Publisher('/brink/out/poly', PolygonStamped, queue_size=10)
hull_pub = rospy.Publisher('/brink/out/hull', PolygonStamped, queue_size=10)
lines_pub = rospy.Publisher('/brink/out/lines', Marker, queue_size=10)
range_pub = rospy.Publisher('/brink/out/range', Float64, queue_size=10)
range_text_pub = rospy.Publisher('/brink/out/range_text', Marker, queue_size=10)
tidied_pub = rospy.Publisher('/brink/out/tidied', PointCloud2, queue_size=10)
map_pub = rospy.Publisher('/brink/out/slope_map', OccupancyGrid, queue_size=10)

# X is going away from camera, Y to left
height = 1.0  # m

x_min = 0.0
x_max = 1.5
width = x_max - x_min

cell_size = 1/16
numY = np.ceil(height/cell_size).astype(int)
y_bins = np.linspace(0., height, num=numY) - height*0.5

numX = np.ceil(width/cell_size).astype(int)
x_bins = np.linspace(x_min, x_max, num=numX)

m = MapMetaData()
m.resolution = cell_size
m.width = numX
m.height = numY
m.origin = Pose()
m.origin.position.x = x_bins[0]
m.origin.position.y = y_bins[0]
map_meta_data = m

MAX_SLOPE = 10.0 # degrees

def int8ToSlope(x):
    return x.astype(float) - 50 * 0.5

def slopeToInt8(x):
    return np.clip((x * 2 + 50).astype(int), -128, 127)

class tfSource:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def lookup(self, _from, _to):
        try:
            tf = self.tfBuffer.lookup_transform(_from, _to, rospy.Time()).transform
            quat = Quaternion(tf.rotation.w,tf.rotation.x,tf.rotation.y,tf.rotation.z)
            trans = np.array([tf.translation.x,tf.translation.y,tf.translation.z])
            return quat, trans
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None


class Brinkmanship:
    def __init__(self, odom_frame_id, filter_size=[0.05, 0.05, 0.05]):
        self.tf_source = tfSource()
        self.odom_frame_id = odom_frame_id
        self.filter_size = filter_size
        self.cloud_sub = rospy.Subscriber('/brink/in/cloud', PointCloud2, self.cloud_handler)

    def msg2np(self, msg):
        pc = ros_numpy.numpify(msg)
        h  = pc.shape[0]
        w  = pc.shape[1]
        pts = np.zeros((h*w, 3), dtype=np.float32)
        pts[:,0] = np.array(pc['x']).flatten()
        pts[:,1] = np.array(pc['y']).flatten()
        pts[:,2] = np.array(pc['z']).flatten()
        return pts

    def np2msg(self, pts):
        h = pts.shape[0]
        w = pts.shape[1]
        data = np.zeros(pts.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        data['x'] = pts[:,0]
        data['y'] = pts[:,1]
        data['z'] = pts[:,2]
        msg = ros_numpy.msgify(PointCloud2, data)
        return msg

    def orient_cloud(self, pts, odom_frame_id, camera_frame_id):
        """ This is supposed to transform the points into the frame of the robot? """
        # If so, we really want to transform points from camera to robot-world oriented
        tf = self.tf_source.lookup(odom_frame_id, camera_frame_id)

        if tf is None:
            print("Brinkmanship failed to find transform from %s to %s." % (camera_frame_id, odom_frame_id))
            # Better handle this...
            return None

        R = tf[0].rotation_matrix
        T = tf[1]

        h = pts.shape[0]
        w = pts.shape[1]
        # NOTE: Seems like this was designed for an ordered PC, but maybe the voxel grid
        # filter unorders things (or its just to_array causing the issue``)
        # pts = pts.reshape((w*h, 3))
        pts = np.dot(R, pts.transpose()).transpose()
        pts += T
        # pts = pts.reshape((h,w,3))
        return pts

    def ray_plane_intersection(self, ray_orig, ray_dir, coeffs):
        normal = coeffs[0:3]
        t = -(ray_orig.dot(normal) + coeffs[3]) / ray_dir.dot(normal)
        return ray_dir*t + ray_orig

    def project_from_origin_to_plane(self, pts, coeffs):
        ray_orig = np.array([0.0, 0.0, 0.0])
        new_pts = np.empty(pts.shape)
        for i,pt in enumerate(pts):
            ray_dir = (pt-ray_orig)/np.linalg.norm(pt-ray_orig)
            new_pts[i,:] = self.ray_plane_intersection(ray_orig, ray_dir, coeffs)
        return new_pts

    def project_point_to_plane(self, pt, plane):
        n = plane[0:3]
        d = plane[3]
        return pt - (n*pt + d) * n

    def cloud_handler(self, msg):
        # Save for posterity.
        camera_frame_id = msg.header.frame_id

        # Convert ros PointCloud2 msg to numpy array.
        pts = self.msg2np(msg)

        # Range filter
        ranges = np.linalg.norm(pts, axis=1)
        mask = np.logical_and(0.02 < ranges, ranges < 1.0)
        pts = pts[mask,:]

        # Convert to pcl PointCloud
        pcl_pts = pcl.PointCloud()
        pcl_pts.from_array(pts)

        # Apply VoxelGrid filter
        f = pcl_pts.make_voxel_grid_filter()
        f.set_leaf_size(self.filter_size[0], self.filter_size[1], self.filter_size[2])
        pcl_pts = f.filter()

        # Give up if the point cloud is empty
        if pcl_pts.size < 10:
            # print("PC empty")
            return

        oriented_pts = self.orient_cloud(pcl_pts.to_array(), self.odom_frame_id, camera_frame_id)
        if oriented_pts is None:
            return
        oriented_pcl = pcl.PointCloud()
        oriented_pcl.from_array(oriented_pts.astype(np.float32))
        tidied_msg = self.np2msg(oriented_pts)
        tidied_msg.header = msg.header
        tidied_msg.header.frame_id = self.odom_frame_id
        tidied_pub.publish(tidied_msg)

        # https://pcl.readthedocs.io/en/latest/normal_estimation.html#normal-estimation
        # Compute normals (slope) of downsampled point cloud
        # TODO: maybe this should be ran on the non downsampled PC
        search_radius = 0.15;
        norm = oriented_pcl.make_NormalEstimation()
        tree = oriented_pcl.make_kdtree()
        norm.set_SearchMethod(tree)
        norm.set_RadiusSearch(search_radius)
        normals = norm.compute().to_array()
        #angle with respect to z unit vector
        angles = np.arccos(normals[:, 2] / np.linalg.norm(normals[:,:3], axis=1))

        xInds = np.digitize(oriented_pts[:,0], x_bins)
        yInds = np.digitize(oriented_pts[:,1], y_bins)

        # Save our slopes in a map
        slopes = np.zeros((numY, numX))
        counts = np.zeros_like(slopes)

        # Bin the slopes by the pts positions
        for xI, yI in zip(xInds, yInds):
            if xI >= numX or yI >= numY or counts[yI, xI] != 0:
                continue

            inds = np.logical_and(xInds == xI, yInds == yI)
            inds = np.logical_and(inds, np.logical_not(np.isnan(angles)))

            cellAngles = angles[inds]
            slopes[yI, xI] = np.mean(cellAngles)
            counts[yI, xI] = inds.shape[0]
        degrees = np.rad2deg(slopes)
        mapSlopes = slopeToInt8(degrees)
        mapFlat = mapSlopes.flatten()

        # Publish the map
        gridMsg = OccupancyGrid()
        gridMsg.header = tidied_msg.header
        gridMsg.info = map_meta_data
        gridMsg.data = mapFlat
        map_pub.publish(gridMsg)

        goodLines = np.sum(counts, axis=1) > 1
        valid = degrees[goodLines,:]
        validCells = counts[goodLines, :] != 0
        safe = np.logical_and(valid < MAX_SLOPE, validCells)
        minDists = np.argmax(safe, axis=1)
        cols = np.indices(valid.shape)[1]
        safe[cols < minDists[:,None]] = True # Extend safe to closer
        distances = x_bins[np.argmin(safe, axis=1)]
        brink_range = distances.min()
        self.pubRange(brink_range)

    def brinkOld(self, pcl_pts, msg):
        camera_frame_id = msg.header.frame_id

        # RANSAC fit a plane
        seg = pcl_pts.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        seg.set_distance_threshold(0.15)

        try:
          indices, model = seg.segment()
          model = np.array(model)
        except:
          print("Brinkmanship: Failed to segment model.")
          return

        if( model[2] > 0.0 ):
            model *= -1

        # Publish the plane polygon
        (A, B, C, D) = (model[0], model[1], model[2], model[3])
        ctr = np.mean(pcl_pts, axis=0)

        poly_msg = PolygonStamped()
        poly_msg.header = msg.header
        poly_msg.header.frame_id = camera_frame_id
        poly_msg.polygon.points.append(Point(-1, -1, (-A-B+D)/-C))
        poly_msg.polygon.points.append(Point(-1, +1, (-A+B+D)/-C))
        poly_msg.polygon.points.append(Point(+1, +1, (+A+B+D)/-C))
        poly_msg.polygon.points.append(Point(+1, -1, (+A-B+D)/-C))
        poly_pub.publish(poly_msg)

        # Smash all points into the plane.
        pts = self.project_from_origin_to_plane(np.asarray(pcl_pts), model)

        # Publish the plane cloud
        cloud_msg = self.np2msg(pts)
        cloud_msg.header = msg.header
        cloud_msg.header.frame_id = camera_frame_id
        cloud_pub.publish(cloud_msg)

        # Move planar points to 2d.
        orig = self.project_point_to_plane(np.array([0,0,0]), model)
        e0 = self.project_point_to_plane(np.array([1,0,0]), model) - orig
        e2 = model[0:3]
        e1 = np.cross(e2, e0)
        basis = np.zeros((3,3))
        basis[:,0] = e0 / np.linalg.norm(e0)
        basis[:,1] = e1 / np.linalg.norm(e1)
        basis[:,2] = e2 / np.linalg.norm(e2)
        pts -= orig
        pts = (basis.transpose().dot(pts.transpose())).transpose() # These transposes are B.S., but it works...
        pts_2d = pts[:,0:2]

        # Compute the inverse for later
        inv_basis = np.linalg.inv(basis)

        # Compute 2d alpha shape
        concave_hull, edge_points = alphashape(pts_2d, alpha=0.1)

        # Draw lines through the alpha shape to see how far you can drive.
        lines = []
        for angle in range(-20, 30+1, 10):
            ro = np.array([0,0.0])
            rd = 10*np.array([np.sin(angle*np.pi/180.0),np.cos(angle*np.pi/180.0)])
            line = LineString([(ro[0]-rd[0], ro[1]-rd[1]), (ro[0]+rd[0], ro[1]+rd[1])])
            int_line = concave_hull.intersection(line)
            try:
                lines.append(int_line.coords)
            except:
                pass

        lines_msg = Marker()
        lines_msg.header = msg.header
        lines_msg.header.frame_id = camera_frame_id
        lines_msg.type = Marker.LINE_LIST
        lines_msg.pose.orientation.w = 1.0
        lines_msg.scale.x = .01; # Line width
        lines_msg.color.r = 1.0; # Use red lines
        lines_msg.color.a = 1.0; # Use opaque lines
        for l in lines:
            xyz = np.array([l[0][0],l[0][1],0]).transpose()
            xyz = xyz.dot(inv_basis)
            xyz += orig
            p0 = Point(xyz[0], xyz[1], xyz[2])
            lines_msg.points.append(p0)

            xyz = np.array([l[1][0],l[1][1],0]).transpose()
            xyz = xyz.dot(inv_basis)
            xyz += orig
            p1 = Point(xyz[0], xyz[1], xyz[2])
            lines_msg.points.append(p1)
        lines_pub.publish(lines_msg)

        try:
            # Publish the estimated range to a brink.
            dists = [np.linalg.norm(np.array([l[0][0]-l[1][0],l[0][1]-l[1][1]])) for l in lines]
            brink_range = np.min(dists)

            self.pubRange(brink_range)

            # Put 2d alpha shape back in the camera_frame and publish it as a polygon.
            hull_msg = PolygonStamped()
            hull_msg.header = msg.header
            hull_msg.header.frame_id = camera_frame_id

            xs = concave_hull.exterior.coords.xy[0]
            ys = concave_hull.exterior.coords.xy[1]
            for x, y in zip(xs, ys):
                xyz = np.array([x, y, 0]).transpose()
                xyz = xyz.dot(inv_basis)
                xyz += orig
                hull_msg.polygon.points.append(Point32(xyz[0], xyz[1], xyz[2]))
            hull_pub.publish(hull_msg)
        except:
            pass

    def pubRange(self, brink_range):
        range_pub.publish(brink_range)

        # Publish a string version of the estimated range to a brink (for rviz).
        range_text_msg = Marker()
        range_text_msg.header.frame_id = "base_link"
        range_text_msg.type = 9

        # Normally white.
        range_text_msg.color.r = 1.0
        range_text_msg.color.g = 1.0
        range_text_msg.color.b = 1.0
        range_text_msg.color.a = 1.0

        # Yellow if getting worried. Red if way too close!
        if brink_range < 0.2:
            range_text_msg.color.g = 0.0
            range_text_msg.color.b = 0.0
        elif brink_range < 0.5:
            range_text_msg.color.b = 0.0

        range_text_msg.scale.z = 0.25
        range_text_msg.pose.position.z = 1.0
        range_text_msg.text = "BRINK: {:03f} m".format(brink_range)
        range_text_pub.publish(range_text_msg)


if __name__=="__main__":
    rospy.init_node('brink')

    brink = Brinkmanship(odom_frame_id='base_link', filter_size=[0.025,0.025,0.025])

    rospy.spin()
