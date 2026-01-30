from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'clio_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), (glob('launch/*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iwintern',
    maintainer_email='cyx2582205445@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
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
