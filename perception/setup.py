from setuptools import setup

package_name = 'perception_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Senior Robotics Architect',
    maintainer_email='architect@marysia.app',
    description='Python-based perception node with artificial latency for Swarm Digital Twin',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = perception_core.detector:main',
            'search_planner = perception_core.search_planner:main',
            'object_localizer = perception_core.object_localizer:main'
        ],
    },
)
