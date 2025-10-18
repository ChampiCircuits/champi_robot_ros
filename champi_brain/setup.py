from setuptools import find_packages, setup

package_name = 'champi_brain'

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
    maintainer='etienne',
    maintainer_email='',
    description='The brain of the Champi robot, handling world state and decision making.',
    license='Driving license for regular mushroom-fueled vehicles.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
