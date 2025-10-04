#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

class SafetyNode(Node):
    """
    Nodo de seguridad final.
    Lee comandos de evasión, policy y smooth, y decide si los ejecuta o los detiene
    en función del láser y la parada de emergencia.
    """

    def __init__(self):
        super().__init__('safety_node')

        # ====== Parámetros fijos ======
        self.input_topics = ['/cmd_smooth', '/cmd_policy', '/cmd_evasion']
        self.output_topic = '/cmd_safety'
        self.scan_topic = '/scan'
        self.estop_topic = '/e_stop'

        # ====== Configuración ======
        self.cmd_timeout_s = 0.5      # tiempo máximo sin recibir comandos
        self.scan_timeout_s = 1.0     # tiempo máximo sin recibir láser
        self.stop_distance_m = 0.4    # distancia crítica (parada)
        self.slow_distance_m = 0.8    # distancia de reducción
        self.slow_min_scale = 0.3     # reducción mínima (30%)
        self.latch_estop = True       # mantener E-Stop hasta reset
        self.publish_rate_hz = 20.0   # frecuencia de publicación

        # ====== Estado interno ======
        self.latest_cmds = {}         # topic → {'twist': Twist, 'stamp': Time}
        self.min_front_range = None
        self.scan_stamp = None
        self.estop_active = False
        self.estop_latched = False

        # ====== QoS para LIDAR ======
        qos_scan = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ====== Suscripciones ======
        for topic in self.input_topics:
            self.create_subscription(Twist, topic, self._make_cmd_cb(topic), 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_scan)
        self.create_subscription(Bool, self.estop_topic, self.estop_cb, 10)

        # ====== Publicadores ======
        self.pub_cmd = self.create_publisher(Twist, self.output_topic, 10)
        self.pub_state = self.create_publisher(String, '/safety/state', 10)

        # ====== Timer principal ======
        period = 1.0 / self.publish_rate_hz
        self.create_timer(period, self.tick)

        self.get_logger().info("Nodo safety iniciado correctamente")

    # ---------- Callbacks ----------
    def _make_cmd_cb(self, topic_name):
        def cb(msg):
            self.latest_cmds[topic_name] = {'twist': msg, 'stamp': self.get_clock().now()}
        return cb

    def scan_cb(self, msg):
        self.scan_stamp = self.get_clock().now()
        self.min_front_range = self._get_min_front(msg, 30.0)  # ±30°

    def estop_cb(self, msg):
        self.estop_active = msg.data
        if self.latch_estop and self.estop_active:
            self.estop_latched = True
        if self.latch_estop and not self.estop_active and self.estop_latched:
            self.estop_latched = False

    # ---------- Lógica principal ----------
    def tick(self):
        now = self.get_clock().now()
        cmd_out = Twist()
        state = "OK"

        # 1️⃣ Selección del comando más reciente y válido
        selected = None
        for topic in self.input_topics:
            rec = self.latest_cmds.get(topic)
            if not rec:
                continue
            age = (now - rec['stamp']).nanoseconds / 1e9
            if age <= self.cmd_timeout_s:
                selected = rec['twist']
                break
        if selected:
            cmd_out = self._copy_twist(selected)
        else:
            state = "NO_CMD"

        # 2️⃣ Reglas de seguridad
        # --- Parada de emergencia
        if self.estop_active or self.estop_latched:
            cmd_out = Twist()
            state = "ESTOP"

        # --- Falta de LIDAR
        elif (self.scan_stamp is None) or ((now - self.scan_stamp).nanoseconds / 1e9 > self.scan_timeout_s):
            cmd_out = Twist()
            state = "NO_SCAN"

        # --- Detección de obstáculo
        elif self.min_front_range is not None:
            d = self.min_front_range
            if d <= self.stop_distance_m:
                cmd_out = Twist()
                state = "OBSTACLE_STOP"
            elif d <= self.slow_distance_m:
                scale = self._interp_scale(d)
                if cmd_out.linear.x > 0.0:
                    cmd_out.linear.x *= scale
                state = "OBSTACLE_SLOW"

        # 3️⃣ Publicación
        self.pub_cmd.publish(cmd_out)
        self.pub_state.publish(String(data=state))

    # ---------- Funciones auxiliares ----------
    def _copy_twist(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = msg.angular.z
        return out

    def _get_min_front(self, scan, half_angle_deg):
        half = math.radians(half_angle_deg)
        start = -half
        end = half
        idx_min = int((start - scan.angle_min) / scan.angle_increment)
        idx_max = int((end - scan.angle_min) / scan.angle_increment)
        idx_min = max(0, idx_min)
        idx_max = min(len(scan.ranges) - 1, idx_max)

        values = [r for r in scan.ranges[idx_min:idx_max] if math.isfinite(r) and r > 0.0]
        return min(values) if values else None

    def _interp_scale(self, d):
        if d <= self.stop_distance_m:
            return 0.0
        if d >= self.slow_distance_m:
            return 1.0
        frac = (d - self.stop_distance_m) / (self.slow_distance_m - self.stop_distance_m)
        return self.slow_min_scale + (1.0 - self.slow_min_scale) * frac


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
