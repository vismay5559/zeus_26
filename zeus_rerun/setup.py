from setuptools import find_packages, setup

package_name = 'zeus_rerun'

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
    description='Rerun visualisation of the Zeus state stream.',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'rerun_node = zeus_rerun.rerun_node:main',
            'rerun_serial = zeus_rerun.rerun_serial:main',
        ],
    },
)
