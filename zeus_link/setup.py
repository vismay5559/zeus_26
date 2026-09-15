from setuptools import find_packages, setup

package_name = 'zeus_link'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vismay',
    maintainer_email='vismayshah121@gmail.com',
    description='USB link to the STM32: 1 kHz state in, residual commands out.',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'link_node = zeus_link.link_node:main',
            'gait_passthrough_node = zeus_link.gait_passthrough_node:main',
            'link_check = zeus_link.link_check:main',
            'fake_board = zeus_link.fake_board:main',
        ],
    },
)
