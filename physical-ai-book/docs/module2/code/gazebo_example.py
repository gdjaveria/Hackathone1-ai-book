import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Pose
import os

class GazeboSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, waiting again...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('delete_entity service not available, waiting again...')

        self.get_logger().info('Gazebo Spawner node ready.')

    def spawn_robot(self, name, xml_path, x=0.0, y=0.0, z=0.0):
        with open(xml_path, 'r') as xml_file:
            robot_xml = xml_file.read()

        request = SpawnEntity.Request()
        request.name = name
        request.xml = robot_xml
        request.robot_namespace = name
        request.initial_pose = Pose()
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(z)

        self.get_logger().info(f'Attempting to spawn entity {name} at ({x}, {y}, {z})')
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Spawn service response: {future.result().status_message}')
        else:
            self.get_logger().error('Spawn service call failed.')

    def delete_robot(self, name):
        request = DeleteEntity.Request()
        request.name = name
        self.get_logger().info(f'Attempting to delete entity {name}')
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Delete service response: {future.result().status_message}')
        else:
            self.get_logger().error('Delete service call failed.')

def main(args=None):
    rclpy.init(args=args)
    spawner = GazeboSpawner()

    # Define path to the URDF file
    # Ensure 'simple_robot.urdf' is in the same directory or provide a full path
    urdf_file_path = os.path.join(os.path.dirname(__file__), 'simple_robot.urdf')

    # Example: Spawn a simple robot
    spawner.spawn_robot('my_simple_robot', urdf_file_path, z=1.0) # Spawn 1 meter above ground

    # Keep the node alive for a bit or until a signal is received
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        spawner.get_logger().info('Keyboard Interrupt received, shutting down.')
    finally:
        # Example: Delete the robot on shutdown
        spawner.delete_robot('my_simple_robot')
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
