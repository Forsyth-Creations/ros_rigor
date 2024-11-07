from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hermes_swerve_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        # install yaml parameter files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='forsythcreations',
    maintainer_email='robert.h.forsyth@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hermes_swerve_module = hermes_swerve_module.swerve_module:main'
        ],
    },
)
