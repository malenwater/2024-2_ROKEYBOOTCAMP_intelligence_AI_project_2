# hangman_game/word_service.py

import rclpy
from rclpy.node import Node
from military_interface.srv import MoveCar
# selectid

class SrvReadyRequestSolveAMRNode(Node):

    def __init__(self,mainServer):
        super().__init__("SrvReadyRequestSolveAMRNode")
        self.get_logger().info('SrvReadyRequestSolveAMRNode start')
        self.ReadyRequest_service_client = self.create_client(
            MoveCar,
            'move_car')

        while not self.ReadyRequest_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The ReadyObjectionSeletion service not available.')
            
        self.mainServer = mainServer
        self.get_logger().info('SrvReadyRequestSolveAMRNode end')
    
    def select_request(self):
        service_request = MoveCar.Request()
        futures = self.ReadyRequest_service_client.call(service_request)
        self.get_logger().info(f'end select_request')
        return futures

