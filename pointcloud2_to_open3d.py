import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('pc2_to_open3d_demo')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(PointCloud2, '/velodyne_points', self.cb, qos)

    def cb(self, msg: PointCloud2):
        try:
            pcd = self.pointcloud2_to_o3d(msg)

        except Exception as e:
            self.get_logger().warn(f"conversion failed in cb: {e}")

    def pointcloud2_to_o3d(self, msg: PointCloud2) -> o3d.geometry.PointCloud:
        want_fields = ('x','y','z')

        xs, ys, zs = [], [], []

        # read_points는 tuple-류를 뱉습니다. 인덱스로 우선 접근, 실패시 getattr로 보조.
        for r in pc2.read_points(msg, field_names=want_fields, skip_nans=True): # r format : (x,y,z,intensity)
            try:
                # tuple 인덱싱 경로
                x, y, z = r[0], r[1], r[2]
                xs.append(x); ys.append(y); zs.append(z)
                
            except Exception:
                # 혹시 namedtuple/np.void 등일 때
                x, y, z = getattr(r, 'x'), getattr(r, 'y'), getattr(r, 'z')
                xs.append(x); ys.append(y); zs.append(z)
                

        # 포인트 구성
        if len(xs) == 0:
            return o3d.geometry.PointCloud()

        xyz = np.column_stack([xs, ys, zs]).astype(np.float64, copy=False)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        return pcd

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
