#!/usr/bin/env python

from setuptools import setup, find_packages

args = dict(
    name='cvra_rpc',
    version='0.2',
    description='Simple RPC protocol for CVRA robots',
    packages=['.'],
    install_requires=['msgpack-python'],
    author='Antoine Albertelli',
    author_email='a.albertelli@cvra.ch',
    url='https://github.com/cvra',
    license='BSD'
)

setup(**args)
