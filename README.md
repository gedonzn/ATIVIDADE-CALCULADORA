# ATIVIDADE ROS2 CALCULADORA
íncrona em ROS 2, que é baseada em um modelo de requisição e resposta. Ao contrário dos tópicos, que são "jogue e esqueça", os serviços são ideais para tarefas que precisam de uma confirmação ou um resultado, como uma chamada de função remota.

# Passo 1: Preparar o Ambiente
Primeiro, você deve criar um workspace (espaço de trabalho) para ROS 2 e navegar até o diretório src.

```terminal
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

# Passo 2: Criar o Pacote ROS 2
Crie o pacote ROS 2 chamado calculadora usando ament_python para indicar que será um pacote Python.

```terminal
ros2 pkg create --build-type ament_python calculadora --dependencies rclpy example_interfaces
```

# Passo 3: Adicionar o Código dos Nós
Navegue para a subpasta de código do pacote e adicione os scripts para o servidor e o cliente.

```terminal
cd ~/ros2_ws/src/calculadora/calculadora/
```

# Código do calculator_server.py (Servidor):
Use um editor de texto de linha de comando como nano para criar o arquivo e cole o código.


```terminal
nano calculator_server.py
```

```python
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
```

# Código do calculator_client.py (Cliente):
Use o mesmo procedimento para criar o arquivo do cliente.

```terminal
nano calculator_client.py
```

```terminal
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
```

# Passo 4: Configurar o setup.py
Volte para a pasta raiz do pacote para editar o arquivo 
setup.py e adicionar os executáveis dos seus nós.

```terminal
cd ~/ros2_ws/src/calculadora/
```

```terminal
nano setup.py
```

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'calculadora'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guilherme-enrique',
    maintainer_email='guilherme-enrique@todo.todo',
    description='Pacote para a Atividade 2 (Serviços e Clientes)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calculator_server = calculadora.calculator_server:main',
            'calculator_client = calculadora.calculator_client:main',
        ],
    },
)
```
# Passo 5: Compilar e Executar
Volte para a raiz do seu workspace e use o colcon build para compilar o pacote.

```terminal
cd ~/ros2_ws/
colcon build --packages-select calculadora
```

# Passo 6: Executar os Nós
Abra terminais separados para executar o servidor e o cliente. Lembre-se de sempre carregar o ambiente ROS 2 em cada novo terminal.
Terminal 1: Execute o Servidor

```terminal
source install/setup.bash
ros2 run calculadora calculator_server
```

Terminal 2: Execute o Cliente
```terminal
source install/setup.bash
ros2 run calculadora calculator_client 5 3
```

Você verá a comunicação em tempo real, com o cliente enviando uma requisição e o servidor retornando a resposta com o resultado da soma.
