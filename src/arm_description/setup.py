import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='Robot URDF and mesh assets for RoboArm.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
