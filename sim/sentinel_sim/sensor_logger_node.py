"""
sensor_logger_node.py
─────────────────────
Subscribes to /sensor_readings (std_msgs/String, JSON payload).
Appends each reading as a row in a CSV file.

Output CSV schema:
    timestamp, rack_id, zone, temperature_c, humidity_pct,
    power_draw_kw, noise_db, arrival_time_s, anomaly_injected, anomaly_type

arrival_time_s is sim clock seconds elapsed since node startup.
"""

import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_OUTPUT = '/tmp/patrol_log.csv'

FIELDNAMES = [
    'timestamp', 'rack_id', 'zone',
    'temperature_c', 'humidity_pct', 'power_draw_kw', 'noise_db',
    'arrival_time_s', 'anomaly_injected', 'anomaly_type',
]

# Zone map (rack_id → zone) — kept in sync with datacenter_layout.json
RACK_ZONE = {
    'RACK-01': 'ZONE-A', 'RACK-02': 'ZONE-A', 'RACK-03': 'ZONE-A',
    'RACK-04': 'ZONE-B', 'RACK-05': 'ZONE-B', 'RACK-06': 'ZONE-B',
}


class SensorLoggerNode(Node):

    def __init__(self):
        super().__init__('sensor_logger_node')

        self.declare_parameter('output_file', DEFAULT_OUTPUT)
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self._start_time = time.monotonic()
        self._row_count  = 0

        # Open CSV (write mode — fresh file each run)
        self._csv_path = output_file
        self._csv_file = open(self._csv_path, 'w', newline='', buffering=1)
        self._writer = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()

        self._sub = self.create_subscription(
            String,
            '/sensor_readings',
            self._on_reading,
            10,
        )

        self.get_logger().info(f'SensorLoggerNode logging to {self._csv_path}')

    def _on_reading(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad JSON on /sensor_readings: {e}')
            return

        rack_id = data.get('rack_id', 'UNKNOWN')

        row = {
            'timestamp':      data.get('timestamp', ''),
            'rack_id':        rack_id,
            'zone':           RACK_ZONE.get(rack_id, 'UNKNOWN'),
            'temperature_c':  data.get('temperature_c', ''),
            'humidity_pct':   data.get('humidity_pct', ''),
            'power_draw_kw':  data.get('power_draw_kw', ''),
            'noise_db':       data.get('noise_db', ''),
            'arrival_time_s': round(time.monotonic() - self._start_time, 2),
            'anomaly_injected': data.get('anomaly_injected', False),
            'anomaly_type':   data.get('anomaly_type', ''),
        }

        self._writer.writerow(row)
        self._row_count += 1
        self.get_logger().info(
            f'Logged reading #{self._row_count} for {rack_id} '
            f'(T={row["temperature_c"]}°C)'
        )

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(
            f'SensorLoggerNode shut down — {self._row_count} rows written to {self._csv_path}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
