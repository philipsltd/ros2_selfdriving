from setuptools import find_packages, setup

package_name = 'navigation_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filipe',
    maintainer_email='filipe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_bag_recorder_vel = navigation_testing.simple_bag_recorder_vel:main',
            'simple_bag_recorder_lidar = navigation_testing.simple_bag_recorder_lidar:main',
            'bag_recorder_full = navigation_testing.bag_recorder_full:main',
            'predict_nav = navigation_testing.predict_nav:main',
            'bag_converter = navigation_testing.bag_converter:main',
        ],
    },
)
