from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = 'demo2_apriltag_docking'


def package_data(directory):
    base = Path(directory)
    return [
        (str(Path('share') / package_name / path.parent), [str(path)])
        for path in base.rglob('*')
        if path.is_file()
    ]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=(
        [
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/config', glob('config/*.yaml')),
            ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ]
        + package_data('models')
        + package_data('maps')
        + package_data('worlds')
    ),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Quchaosheng',
    maintainer_email='Quchaosheng@users.noreply.github.com',
    description='AprilTag visual docking demo using Nav2 Docking.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_pose_bridge = demo2_apriltag_docking.tag_pose_bridge:main',
            'docking_task_bridge = demo2_apriltag_docking.docking_task_bridge:main',
        ],
    },
)
