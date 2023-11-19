from setuptools import setup

package_name = 'scenario_lane_change_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'scenario_lane_change_test_node = scenario_lane_change_test.scenario_lane_change_test_node:main'
        ],
    },
)
