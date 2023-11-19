from setuptools import setup

package_name = 'coop_localization_py'

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
    description='localization of robot with map matching',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = coop_localization_py.localization:main'
        ],
    },
)
