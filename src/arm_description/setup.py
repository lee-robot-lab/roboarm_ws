from setuptools import find_packages, setup
import os
from glob import glob

# src/arm_description/setup.py

package_name = 'arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), # <-- 이렇게 고쳐주세요 (또는 'resource/arm_description')
        ('share/' + package_name, ['package.xml']),
        
        # URDF와 Mesh 복사 설정 유지
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            
        ],
    },
)
