from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'homer_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "configs"),
            glob(os.path.join("configs", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pbd0',
    maintainer_email='pbd0@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'homer_interface = homer_bringup.homer_interface:main'
        ],
    },
)
