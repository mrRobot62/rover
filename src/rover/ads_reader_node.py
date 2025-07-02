import rclpy
from rclpy.node import Node
from rover_interfaces.srv import I2CReadRequest

#---------------------------------------------------
# Aktuell nur DEMO
#---------------------------------------------------


class ADSReaderNode(Node):
    def __init__(self):
        super().__init__('ads_reader_node')
        self.cli = self.create_client(I2CReadRequest, '/i2c/read')
        self.timer = self.create_timer(1.0, self.read_adc)

    def read_adc(self):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('I2C Read Service nicht verfügbar')
            return

        req = I2CReadRequest.Request()
        req.device = "ads1115"
        future = self.cli.call_async(req)

        def cb(future):
            try:
                result = future.result()
                self.get_logger().info(f"ADS1115 Werte: {result.values}")
            except Exception as e:
                self.get_logger().error(f"Service-Call fehlgeschlagen: {str(e)}")
        future.add_done_callback(cb)