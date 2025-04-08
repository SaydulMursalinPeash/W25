from setuptools import find_packages
from setuptools import setup

setup(
    name='W25',
    version='0.0.0',
    packages=find_packages(
        include=('W25', 'W25.*')),
)
