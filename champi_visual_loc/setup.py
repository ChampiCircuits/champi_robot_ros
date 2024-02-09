from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'champi_visual_loc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*')),
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
            'visual_loc_node = champi_visual_loc.visual_loc_node:main',
        ],
    },
)
