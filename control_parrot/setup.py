from setuptools import find_packages, setup

package_name = 'control_parrot'

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
    maintainer_email='marcosmutzcontato@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'link_analizer = control_parrot.link_analizer:main',
            'control_parrot_indoor = control_parrot.control_parrot_indoor:main',
            'control_parrot_joystick = control_parrot.control_parrot_joystick:main',
            'control_parrot_servovisual = control_parrot.control_parrot_servovisual:main'
        ],
    },
)
