from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_controller'


data_files = [
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
            os.path.join("lib", package_name, "backend"),
            glob(os.path.join("web_controller", "backend", "*.py")),
        ),
        # include the app directory
        (
            os.path.join("lib", package_name, "backend", "app"),
            glob(os.path.join("web_controller", "backend", "app", "*.py")),
        ),
        #include the package.json file
        (
            os.path.join("share", package_name, "frontend"),
            glob(os.path.join("web_controller", "frontend", "package.json")),
        ),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['web_controller/frontend/.next']),
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
