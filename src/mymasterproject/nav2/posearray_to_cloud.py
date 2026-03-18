# posearray_to_cloud.py
import rclpy, math, struct
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class PoseArrayToCloud(Node):
    def __init__(self):
        super().__init__('posearray_to_cloud')
        self.declare_parameter('input_topic', '/actors/poses')
        self.declare_parameter('output_topic', '/actors_cloud')
        # If your PoseArray.header.frame_id is already correct, set force_frame_id="" to reuse it.
        self.declare_parameter('force_frame_id', 'map')  # "map" or "odom" or "" to use incoming
        self.declare_parameter('actor_radius', 0.35)     # meters (tweak)
        self.declare_parameter('points_per_actor', 16)   # 8–24 is fine

        self.input_topic  = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.force_frame  = self.get_parameter('force_frame_id').value
        self.r            = float(self.get_parameter('actor_radius').value)
        self.N            = int(self.get_parameter('points_per_actor').value)

        self.sub = self.create_subscription(PoseArray, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f"Listening to {self.input_topic}, publishing {self.output_topic}")

    def cb(self, msg: PoseArray):
        frame_id = self.force_frame if self.force_frame else msg.header.frame_id or 'map'
        pts_bin = bytearray()
        for p in msg.poses:
            cx, cy = p.position.x, p.position.y
            for k in range(self.N):
                th = 2.0*math.pi*k/self.N
                x = cx + self.r*math.cos(th)
                y = cy + self.r*math.sin(th)
                pts_bin += struct.pack('fff', x, y, 0.0)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        cloud = PointCloud2(
            header=header,
            height=1,
            width=len(pts_bin)//12,
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12*(len(pts_bin)//12),
            data=bytes(pts_bin),
            is_dense=True,
        )
        self.pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(PoseArrayToCloud())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
