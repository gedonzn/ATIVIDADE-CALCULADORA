import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    print(f'Requisição recebida: a={request.a}, b={request.b}')
    print(f'Resposta enviada: sum={response.sum}')
    return response

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node('calculator_server')
    srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
    print('Serviço de soma de inteiros iniciado.')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
