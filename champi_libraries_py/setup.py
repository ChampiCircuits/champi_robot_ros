from setuptools import find_packages, setup

package_name = 'champi_libraries_py'

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
    maintainer='andre',
    maintainer_email='andrerusso02@gmail.com',
    description='A bookcase, that contains mushrooms and snakes instead of actual books.',
    license='Driving license for regular mushroom-fueled vehicles.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
