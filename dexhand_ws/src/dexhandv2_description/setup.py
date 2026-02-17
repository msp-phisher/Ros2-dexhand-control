from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dexhandv2_description'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Install Rviz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        
        # Install Meshes (retaining subfolder structure)
        (os.path.join('share', package_name, 'meshes/right'), glob('meshes/right/*')),
        (os.path.join('share', package_name, 'meshes/cobot_right'), glob('meshes/cobot_right/*')),
        
        # Install Config files (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # NEW: Install Scripts to 'lib' so you can run them with 'ros2 run'
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trent Shumay',
    maintainer_email='trent@iotdesignshop.com',
    description='ROS 2 description package for DexHand v2',
    license='CC BY-NC-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
