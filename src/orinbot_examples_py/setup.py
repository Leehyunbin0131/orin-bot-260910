from setuptools import find_packages, setup

package_name = 'orinbot_examples_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyunlee',
    maintainer_email='unit60888@gmail.com',
    description='rclpy example nodes for orinbot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'square_driver = orinbot_examples_py.square_driver:main',
        ],
    },
)
