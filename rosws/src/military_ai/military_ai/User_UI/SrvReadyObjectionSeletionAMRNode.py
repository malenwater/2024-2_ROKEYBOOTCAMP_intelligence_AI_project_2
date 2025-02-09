# hangman_game/word_service.py

import rclpy
from rclpy.node import Node
from military_interface.srv import SelectID
# selectid

class SrvReadyObjectionSeletionAMRNode(Node):

    def __init__(self,mainServer):
        super().__init__("SrvReadyObjectionSeletionAMRNode")
        self.get_logger().info('SrvReadyObjectionSeletionAMRNode start')
        self.ReadyObjectionSeletion_service_client = self.create_client(
            SelectID,
            'select_ID')

        while not self.ReadyObjectionSeletion_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The ReadyObjectionSeletion service not available.')
            
        self.mainServer = mainServer
        self.get_logger().info('SrvReadyObjectionSeletionAMRNode end')
    
    def select_request(self,data):
        service_request = SelectID.Request()
        service_request.selectid = float(data)
        futures = self.ReadyObjectionSeletion_service_client.call(service_request)
        self.get_logger().info(f'end select_request')
        return futures

