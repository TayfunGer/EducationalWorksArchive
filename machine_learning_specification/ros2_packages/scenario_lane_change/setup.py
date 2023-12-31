from setuptools import setup
from glob import glob
import os

package_name = 'scenario_lane_change'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tayfun',
    maintainer_email='s75218@beuth-hochschule.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_lane_change_node = scenario_lane_change.scenario_lane_change_node:main'
        ],
    },
)
