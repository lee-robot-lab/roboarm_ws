from setuptools import find_packages, setup

package_name = 'roboarm_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='IK and trajectory planning nodes for RoboArm.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ik_planner_node = roboarm_planner.plan_node:main',
            'mit_bridge = roboarm_planner.mit_bridge:main',
        ],
    },
)
