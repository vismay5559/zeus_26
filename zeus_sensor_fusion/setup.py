from setuptools import find_packages, setup

package_name = 'zeus_sensor_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_fusion.launch.py']),
        ('share/' + package_name + '/config', [
            'config/kinematics.yaml',
            'config/inekf_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vismay',
    maintainer_email='vismayshah121@gmail.com',
    description='Contact-aided InEKF state estimator for Zeus bipedal robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = zeus_sensor_fusion.sensor_fusion_node:main',
        ],
    },
)
