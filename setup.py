from setuptools import find_packages, setup
from glob import glob

package_name = 'urdf_to_opw_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeroen',
    maintainer_email='jeroen@todo.todo',
    description='The urdf_to_opw_kinematics package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example = urdf_to_opw_kinematics.example:example',
            'run = urdf_to_opw_kinematics.example:main'
        ],
    },
)
