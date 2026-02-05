import os
from glob import glob
from setuptools import setup

package_name = 'arm_sim'

setup(
    name=package_name,
    version='0.0.0',
    # 파이썬 코드가 다른 곳으로 갔으므로 패키지는 비워둡니다.
    packages=[], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # [핵심] 이제 robot.xml 하나만 설치하면 됩니다.
        (os.path.join('share', package_name), glob('*.xml')),
        
        # [삭제됨] meshes와 urdf는 이제 arm_description에서 관리하므로 제거했습니다.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='MuJoCo simulation configuration package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # [삭제됨] mujoco_mit_bridge는 arm_driver 패키지로 이동했습니다.
        ],
    },
)