from setuptools import setup
import os # Operating system library
from glob import glob # Handles file path names

package_name = 'kick_ass'

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    

]

# Path for folders with subfolders in package
def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

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
    packages=[package_name],

    data_files=package_files(data_files, ['models/', 'launch/', 'worlds/', 'urdf/' , 'rviz/' , 'meshes/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffin',
    maintainer_email='jeffinjohnyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["estimate = kick_ass.robot_estimator:main",
        "control = kick_ass.robot_controller:main" 
        ],
    },
)
