import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def convert_cloud_to_ros2_msg(cloud, frame_id="map"):
    # Extract point cloud data
    points = np.asarray(cloud.points)
    num_points = len(points)

    # Create a header
    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id

    # Create PointCloud2 fields
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Convert the point cloud to PointCloud2 format
    pc2_msg = PointCloud2(
        header=header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=12,
        row_step=12 * num_points,
        data=points.astype(np.float32).tobytes(),
        is_dense=True
    )
    return pc2_msg

def main(args=None):
    rclpy.init(args=args)
    node = Node('ply_to_pointcloud2')

    # Load the .ply file
    pcd = o3d.io.read_point_cloud("/home/olagh/gaussian-splatting/output/56e79fa4-f/point_cloud/iteration_30000/point_cloud.ply")

    # Convert to ROS2 PointCloud2 message
    pc2_msg = convert_cloud_to_ros2_msg(pcd)

    # Publish the message
    pub = node.create_publisher(PointCloud2, 'pointcloud_topic', 10)
    pub.publish(pc2_msg)
    node.get_logger().info("Published point cloud to pointcloud_topic")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
