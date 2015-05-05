#!/usr/bin/env python

from setuptools import setup

args = dict(
    name='master-board',
    version='0.1',
    description='Functions to do basic interactions with the master board',
    packages=['master_board'],
    author='CVRA',
    author_email='info@cvra.ch',
    url='https://github.com/cvra',
    license='BSD'
)

setup(**args)
