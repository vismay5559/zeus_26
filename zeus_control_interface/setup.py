from setuptools import find_packages, setup

package_name = 'zeus_control_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vismay',
    maintainer_email='vismay@todo.todo',
    description='Python Deep RL Policy Node for Zeus',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This links the terminal command to your python script
            'rl_policy_node = zeus_control_interface.rl_policy_node:main'
        ],
    },
)