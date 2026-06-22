from setuptools import find_packages, setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data',
            ['vision_node/haarcascade_frontalface_default.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hangman',
    maintainer_email='pavanmoovaje123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'face_detector= vision_node.face_detector_node:main',
            'vision_monitor= vision_node.vision_monitor_node:main',
            'sim_camera_detector= vision_node.sim_camera_detector:main',
        ],
    },
)
