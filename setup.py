from setuptools import setup
from glob import glob
import os

package_name = '41068_ignition_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ]
    }
)