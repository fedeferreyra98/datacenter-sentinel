"""
virtual_sensor_node.py
─────────────────────
Listens for waypoint-arrival events published by patrol_node on
/patrol/waypoint_reached (std_msgs/String carrying the rack_id).

When triggered, generates a synthetic sensor reading for that rack
and publishes it on /sensor_readings (std_msgs/String, JSON payload).

Sensor values are synthesized using the same statistical model as
notebook 01_data_generator:
  - Temperature : N(22, 1.5) °C  +  optional anomaly spike
  - Humidity    : N(45, 3)   %
  - Power draw  : N(5, 0.5)  kW  per rack
  - Noise level : N(55, 2)   dB
"""

import json
import math
import random
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ── Anomaly injection config ───────────────────────────────────────────────
# For demo purposes, RACK-02 will simulate a T01 (thermal runaway) anomaly.
# Set to None to disable all anomalies.
ANOMALY_RACK = 'RACK-02'
ANOMALY_TYPE = 'T01'  # thermal runaway


class VirtualSensorNode(Node):

    def __init__(self):
        super().__init__('virtual_sensor_node')

        # Track elapsed sim time for gradual anomaly evolution
        self._start_time = time.monotonic()

        # Publishers / Subscribers
        self._pub = self.create_publisher(String, '/sensor_readings', 10)
        self._sub = self.create_subscription(
            String,
            '/patrol/waypoint_reached',
            self._on_waypoint_reached,
            10,
        )

        self.get_logger().info('VirtualSensorNode ready — listening on /patrol/waypoint_reached')

    # ── Callback ──────────────────────────────────────────────────────────

    def _on_waypoint_reached(self, msg: String):
        rack_id = msg.data.strip()
        reading = self._generate_reading(rack_id)

        out = String()
        out.data = json.dumps(reading)
        self._pub.publish(out)

        self.get_logger().info(
            f'[{rack_id}] T={reading["temperature_c"]:.1f}°C  '
            f'H={reading["humidity_pct"]:.1f}%  '
            f'P={reading["power_draw_kw"]:.2f}kW  '
            f'N={reading["noise_db"]:.1f}dB'
            + (' ⚠ ANOMALY' if reading['anomaly_injected'] else '')
        )

    # ── Sensor model ──────────────────────────────────────────────────────

    def _generate_reading(self, rack_id: str) -> dict:
        elapsed_min = (time.monotonic() - self._start_time) / 60.0
        anomaly_injected = False

        # Base values (Gaussian noise around nominal)
        temperature = random.gauss(22.0, 1.5)
        humidity    = random.gauss(45.0, 3.0)
        power_draw  = random.gauss(5.0,  0.5)
        noise_db    = random.gauss(55.0, 2.0)

        # Anomaly injection
        if rack_id == ANOMALY_RACK:
            if ANOMALY_TYPE == 'T01':
                # T01: Thermal runaway — gradual rise over 30–60 min
                rise = min(elapsed_min / 30.0, 1.0) * 18.0  # up to +18°C
                temperature += rise + random.gauss(0, 0.5)
                anomaly_injected = True

            elif ANOMALY_TYPE == 'T02':
                # T02: Fan degradation — periodic noise spikes
                spike = 8.0 * abs(math.sin(elapsed_min * math.pi / 5.0))
                noise_db += spike
                anomaly_injected = True

            elif ANOMALY_TYPE == 'T03':
                # T03: PDU overload — step increase in power + oscillation
                step = 3.0 if elapsed_min > 5.0 else 0.0
                osc  = 0.5 * math.sin(elapsed_min * math.pi / 2.0)
                power_draw += step + osc
                anomaly_injected = True

            elif ANOMALY_TYPE == 'T04':
                # T04: Humidity breach — slow drift past threshold (60%)
                drift = min(elapsed_min / 20.0, 1.0) * 20.0
                humidity += drift
                anomaly_injected = True

        return {
            'timestamp':       datetime.now(timezone.utc).isoformat(),
            'rack_id':         rack_id,
            'temperature_c':   round(max(15.0, temperature), 2),
            'humidity_pct':    round(max(10.0, min(100.0, humidity)), 2),
            'power_draw_kw':   round(max(0.0,  power_draw), 3),
            'noise_db':        round(max(20.0, noise_db), 2),
            'anomaly_injected': anomaly_injected,
            'anomaly_type':    ANOMALY_TYPE if anomaly_injected else None,
        }


def main(args=None):
    rclpy.init(args=args)
    node = VirtualSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
