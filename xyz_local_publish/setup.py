from setuptools import find_packages, setup

package_name = 'xyz_local_publish'

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
    maintainer='tonim',
    maintainer_email='tonim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_read = xyz_local_publish.gps_read:main',
            'pose_read = xyz_local_publish.pose_read:main',
            'optitrack_read = xyz_local_publish.optitrack_read:main'
        ],
    },
)
