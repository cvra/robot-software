from setuptools import setup

setup(
    name='pid_tuner',
    version='0.0.0',
    description='PID tuning tool for CVRA motor boards',
    author='Club Vaudois de Robotique Autonome',
    author_email='info@cvra.ch',
    url='http://cvra.ch',
    license='BSD',
    packages=['pid_tuner'],
    install_requires=[
        'PyQt5',
        'pyserial',
        'pyqtgraph',
        'uavcan',
    ],
    entry_points={
        'console_scripts': [
            'pid_tuner=pid_tuner.pid_tune:main',
            'motor_load_config=pid_tuner.load_initial_config:main',
        ],
    }, )
