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
