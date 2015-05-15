from fabric.api import *


def debra():
    env.hosts += ['192.168.2.10']
    env.user = 'cvra'


def caprica():
    env.hosts += ['192.168.2.20']
    env.user = 'cvra'


def vm():
    env.hosts += ['localhost:2222']
    env.user = 'cvra'


def build():
    local('make -j')


def rebuild():
    local('make clean')
    local('make dsdlc')
    local('packager/packager.py')
    local('make -B -j')


def copy_files():
    run('rm -rf ~/deploy_master')
    run('mkdir ~/deploy_master')
    put('build/ch.elf', '~/deploy_master/ch.elf')
    put('oocd_busblaster_v3.cfg', '~/deploy_master/oocd.cfg')

def serial():
    sudo('python -m serial.tools.miniterm /dev/ttyACM0')


def flash():
    sudo('openocd -f ~cvra/deploy_master/oocd.cfg -c "program /home/cvra/deploy_master/ch.elf verify reset" -c "shutdown"')


def deploy():
    build()
    copy_files()
    flash()
