from fabric.api import *

MASTER_BOARD = {
    'debra': '10.0.10.2',
    'caprica': '10.0.20.2',
}

MOTOR_BOARDS = {
    'debra': [20, 21, 29, 31, 32, 22, 23, 24],  # right: 25, 27, 28
    'caprica': [],
}


def debra():
    env.hosts += ['debra']
    env.user = 'cvra'


def nastya():
    env.hosts += ['nastya']
    env.user = 'cvra'


def localhost():
    env.hosts += ['localhost']


def build():
    """
    Compiles the project.
    """
    local('make dsdlc')
    local('packager/packager.py')
    local('make -j')


def rebuild():
    """
    Makes a clean build of the project from scratch.
    """
    local('make clean')
    local('make dsdlc')
    local('packager/packager.py')
    local('make -B -j')

def reboot():
    """
    Reboots all motor boards connected to the bus.
    """
    command = "python3 reboot_uavcan_nodes.py "
    command += MASTER_BOARD[env.host] + " "
    command += " ".join(map(str, MOTOR_BOARDS[env.host]))
    local(command)

def run():
    """
    Reads the config of all connected boards.
    """
    command = "python3 can-bootloader/client/bootloader_run_application.py"
    command += " --tcp {}".format(MASTER_BOARD[env.host])
    command += " --all"
    local(command)

def read_config():
    """
    Reads the config of all connected boards.
    """
    reboot()
    command = "python3 can-bootloader/client/bootloader_read_config.py"
    command += " --tcp {}".format(MASTER_BOARD[env.host])
    command += " --all"
    local(command)


def deploy():
    """
    Uploads the binary to the robot.
    """
    build()
    reboot()

    flash_command = "python3 can-bootloader/client/bootloader_flash.py"
    # Base adress
    flash_command += " -a 0x08003800"
    flash_command += " --tcp {}".format(MASTER_BOARD[env.host])
    flash_command += " --device-class motor-board-v1"
    flash_command += " -b build/motor-control-firmware.bin"
    flash_command += " --run"
    flash_command += " " + " ".join(str(i) for i in MOTOR_BOARDS[env.host])
    local(flash_command)
