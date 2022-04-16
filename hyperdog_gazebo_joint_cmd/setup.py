from setuptools import setup

package_name = 'hyperdog_gazebo_joint_cmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nd94',
    maintainer_email='nipun.dhananjaya@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hyperdog_gazebo_joint_controller = hyperdog_gazebo_joint_cmd.hyperdog_gazebo_joint_controller:main'
        ],
    },
)
