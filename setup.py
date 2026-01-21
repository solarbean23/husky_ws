from setuptools import find_packages, setup

package_name = 'webots_ros2_husky'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/protos', ['protos/Husky.proto']))
data_files.append(('share/' + package_name + '/protos', ['protos/Fire_UGV.proto']))
data_files.append(('share/' + package_name + '/protos', ['protos/Rescue_UGV.proto']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/husky_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/fire_rescue_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/ros2control.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/husky.urdf']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds/icons', ['worlds/icons/base.png']))
data_files.append(('share/' + package_name + '/worlds/icons', ['worlds/icons/rescue_white.png']))
data_files.append(('share/' + package_name + '/worlds/icons', ['worlds/icons/rescue_red.png']))
data_files.append(('share/' + package_name + '/worlds/icons', ['worlds/icons/target.png']))
data_files.append(('share/' + package_name + '/worlds/icons', ['worlds/icons/mavic_fast_helix.png']))
data_files.append(('share/' + package_name + '/controllers/world_supervisor', ['controllers/world_supervisor/world_supervisor.py']))
data_files.append(('share/' + package_name + '/controllers/robot_supervisor', ['controllers/robot_supervisor/robot_supervisor.py']))
data_files.append(('share/' + package_name + '/protos', ['protos/Mavic2Pro.proto']))
data_files.append(('share/' + package_name + '/controllers/mavic_driver', ['controllers/mavic_driver/mavic_driver.py']))



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email_address',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)