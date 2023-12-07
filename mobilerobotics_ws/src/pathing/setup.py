from setuptools import setup

package_name = 'pathing'

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
    maintainer='april',
    maintainer_email='dyllon.dunton@maine.edu',
    description='pathing with aruco tags',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'planner = pathing.better_soccer:main',
        	'angle = pathing.standard_angle:main',
        	'forward = pathing.standard_movement:main',
        ],
    },
)
