# hangman_game/word_service.py

import rclpy
from rclpy.node import Node
from military_interface.srv import SelectID
# selectid

class SrvObjectionSeletionAMRNode(Node):

    def __init__(self,mainServer):
        super().__init__("SrvObjectionSeletionAMRNode")
        self.get_logger().info('SrvObjectionSeletionAMRNode start')
        self.select_srv = self.create_service(SelectID, 'select_ID', self.select_callback)
        self.mainServer = mainServer
        self.get_logger().info('SrvObjectionSeletionAMRNode end')
        
    def select_callback(self, request, response):
        self.get_logger().info(f'Received select_callback request: {request}')
        select_ID = float(request.selectid)
        self.mainServer.set_follow_car_ID(select_ID)
        self.mainServer.set_AMR_STATUS(0)
        self.get_logger().info(f'end select_callback request')

        return response
