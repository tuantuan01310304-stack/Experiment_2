import os
from glob import glob
from setuptools import setup

package_name = 'mecharm_grasp_lws'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='lws',
    maintainer_email='lws@example.com',

    description='LWS mechArm 270 multi-cycle grasping experiment',

    license='MIT',

    entry_points={
        'console_scripts': [
            'grasp_task_lws = mecharm_grasp_lws.grasp_task_lws:main',
            'lws_transfer_task = mecharm_grasp_lws.lws_transfer_task:main',
            'real_transfer_lws = mecharm_grasp_lws.real_transfer_lws:main',
            'real_cycle_aba_lws = mecharm_grasp_lws.real_cycle_aba_lws:main',
            'real_cycle_5_lws = mecharm_grasp_lws.real_cycle_5_lws:main',
            'real_cycle_5_home_lws = mecharm_grasp_lws.real_cycle_5_home_lws:main',
            'real_cycle_5_home_fast_lws = mecharm_grasp_lws.real_cycle_5_home_fast_lws:main',
            'real_cycle_5_home_fast2_lws = mecharm_grasp_lws.real_cycle_5_home_fast2_lws:main',
            'real_cycle_5_home_fast3_lws = mecharm_grasp_lws.real_cycle_5_home_fast3_lws:main',
        ],
    },
)
