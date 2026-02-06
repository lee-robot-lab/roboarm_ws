import os
from glob import glob
from setuptools import setup

package_name = 'arm_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='MuJoCo XML configuration package for RoboArm.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
