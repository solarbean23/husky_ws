from setuptools import find_packages, setup

package_name = 'webots_ros2_husky'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/protos', ['protos/Husky.proto']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/husky_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/fire_rescue_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/ros2control.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/husky.urdf']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/protos/icons', ['protos/icons/base.jpg']))

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