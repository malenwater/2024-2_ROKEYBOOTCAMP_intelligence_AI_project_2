# hangman_game/word_service.py

import rclpy
from rclpy.node import Node
from military_interface.srv import MoveCar


class RequestSolveNode(Node):

    def __init__(self):
        super().__init__("word_service")
        self.service = self.create_service(
            MoveCar, "check_letter", self.check_letter_callback
        )

    def check_letter_callback(self, request, response):
        letter = self.current_letter
        if letter in self.word:
            for idx, char in enumerate(self.word):
                if char == letter:
                    self.word_state[idx] = letter
            response.is_correct = True
            response.message = "Correct!"
        else:
            self.attempts_left -= 1
            response.is_correct = False
            response.message = "WRONG"
        response.updated_word_state = "".join(self.word_state)
        self.get_logger().info(f"Received letter: {letter}")
        self.get_logger().info(f"Current word state: {response.updated_word_state}")

        return response


def main(args=None):
    rclpy.init(args=args)
    word_service = RequestSolveNode()
    rclpy.spin(word_service)
    word_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
