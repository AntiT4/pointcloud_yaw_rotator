from setuptools import setup

package_name = 'pointcloud_yaw_rotator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Rotate PointCloud2 data around yaw axis',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rotator_node = pointcloud_yaw_rotator.rotator_node:main',
        ],
    },
)
