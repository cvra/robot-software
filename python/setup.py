#!/usr/bin/env python

from setuptools import setup, find_packages

args = dict(
    name='serial_datagram',
    version='0.2',
    description='Transmit datagrams over stream oriented transports',
    packages=['.'],
    author='CVRA',
    author_email='info@cvra.ch',
    url='https://github.com/cvra',
    license='BSD'
)

setup(**args)
