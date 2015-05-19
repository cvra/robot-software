from fabric.api import *
import time


def debra():
    """
    Setup the environment to use it on Debra.
    """
    env.hosts += ['192.168.2.10']
    env.user = 'cvra'


def caprica():
    """
    Setup the environment to use it on caprica.
    """
    env.hosts += ['192.168.2.20']
    env.user = 'cvra'


def vm():
    env.hosts += ['localhost:2222']
    env.user = 'cvra'


def build():
    """
    Compiles the project.
    """
    local('make dsdlc')
    local('packager/packager.py')
    local('make -j')


def rebuild():
    """
    Makes a clean build of the project.
    """
    local('make clean')
    local('make dsdlc')
    local('packager/packager.py')
    local('make -B -j')


def _copy_files():
    run('rm -rf ~/deploy_master')
    run('mkdir ~/deploy_master')
    put('build/ch.elf', '~/deploy_master/ch.elf')
    put('oocd_busblaster_v3.cfg', '~/deploy_master/oocd.cfg')


def serial():
    """
    Opens a serial terminal to the master board.
    """
    sudo('python -m serial.tools.miniterm /dev/ttyACM*')


def _flash():
    sudo('openocd -f ~cvra/deploy_master/oocd.cfg -c "program /home/cvra/deploy_master/ch.elf verify reset" -c "shutdown"')


def deploy():
    """
    Deploys the firmware on the given robot.
    """
    build()
    _copy_files()
    _flash()

def test():
    i = 0
    while True:
        print("Test {}".format(i))
        i = i + 1
        local("python config_send.py  config.yaml 10.0.10.2")
        time.sleep(1.)
