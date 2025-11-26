from setuptools import setup

package_name = 'game_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/game.launch.py',
            'launch/oakd_game.launch.py',
            'launch/oakd_stereo.launch.py',
            'launch/stereo_with_rviz.launch.py',
            'launch/oak_only.launch.py',     # <-- ADDED
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email',
    description='Game nodes for navigation and obstacle detection',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'navigator = game_nodes.navigator:main',
            'obstacle_detector = game_nodes.obstacle_detector:main',
            'oakd_stereo = game_nodes.oakd_stereo:main',
            'controller = game_nodes.controller:main'
        ],
    },
)
