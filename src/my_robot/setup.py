from setuptools import find_packages, setup

package_name = 'my_robot'

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
            'square_driver = my_robot.square_driver:main',  # Add this line for the square_driver node
            'obstacle_avoidance = my_robot.obstacle_avoidance:main',    # Add this line for the obstacle_avoidance node
            'target_reactor = my_robot.target_reactor:main',  # Add this line for the target_reactor node
        ],
    },
)
