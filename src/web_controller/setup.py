from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_controller'

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
        # include all the python files.
        (
            os.path.join("lib", package_name),
            glob(os.path.join("web_controller", "**", "*.py")),
        ),
    ],
    install_requires=['setuptools', 'uvicorn', 'fastapi', 'pydantic'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
