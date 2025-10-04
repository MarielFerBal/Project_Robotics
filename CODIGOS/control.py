import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Últimos valores recibidos
        self.evasion_val = 0.0
        self.velocity_policy_val = 0.0

        # Suscriptores
        self.sub_evasion = self.create_subscription(
            Float32, '/evasion', self.evasion_callback, 10)
        self.sub_velocity_policy = self.create_subscription(
            Float32, '/velocity_policy', self.velocity_callback, 10)

        # Publicador
        self.pub_velocity = self.create_publisher(Float32, '/salida_velocidad', 10)

        self.get_logger().info("Nodo iniciado: suscribiendo a /evasion y /velocity_policy")

    def evasion_callback(self, msg):
        self.evasion_val = msg.data
        self.get_logger().debug(f"Recibido evasion: {self.evasion_val}")

    def velocity_callback(self, msg):
        self.velocity_policy_val = msg.data
        self.get_logger().info(f"Recibido velocity_policy: {self.velocity_policy_val}")

        # Lógica simple: por ejemplo, multiplicamos por 0.5 si evasion es alta
        velocidad_salida = Float32()
        if self.evasion_val > 5.0:
            velocidad_salida.data = self.velocity_policy_val * 0.5
        else:
            velocidad_salida.data = self.velocity_policy_val

        # Publicar velocidad
        self.pub_velocity.publish(velocidad_salida)
        self.get_logger().info(f"Publicado velocidad: {velocidad_salida.data}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
