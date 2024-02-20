from setuptools import find_packages, setup

package_name = 'dev_tools'

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
    maintainer='arusso',
    maintainer_email='andrerusso02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holo_teleop_joy = dev_tools.holo_teleop_joy:main',
            'camera_info_publisher = dev_tools.camera_info_publisher:main'
        ],
    },
)
