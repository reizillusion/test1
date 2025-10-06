from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'order_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='illusion',
    maintainer_email='illusion0121@163.com',
    description='Cafe order simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'OrderManager_node = order_py_pkg.OrderManager:main',
            'BaristaBot_node = order_py_pkg.BaristaBot:main',
            'Customer_node = order_py_pkg.customer:main',
        ],
    },
)