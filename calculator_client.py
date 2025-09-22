import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print('Uso: ros2 run calculadora calculator_client <arg1> <arg2>')
        return
    
    node = Node('calculator_client')
    cli = node.create_client(AddTwoInts, 'add_two_ints')
    
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Serviço não disponível, esperando...')
        
    req = AddTwoInts.Request()
    req.a = int(sys.argv[1])
    req.b = int(sys.argv[2])
    
    future = cli.call_async(req)
    
    rclpy.spin_until_future_complete(node, future)
    
    try:
        response = future.result()
        node.get_logger().info(f'Resultado da soma: {response.sum}')
    except Exception as e:
        node.get_logger().error(f'Falha na chamada do serviço: {e}')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
