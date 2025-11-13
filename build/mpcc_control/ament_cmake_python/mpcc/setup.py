from setuptools import find_packages
from setuptools import setup

setup(
    name='mpcc',
    version='0.0.0',
    packages=find_packages(
        include=('mpcc', 'mpcc.*')),
)
