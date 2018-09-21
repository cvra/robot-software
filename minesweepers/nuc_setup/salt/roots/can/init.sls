can-utils:
  pkg.installed

slcan_interface_service:
  file.managed:
    - name: /etc/systemd/system/slcan.service
    - source: salt://can/slcan.service
  service.running:
    - name: slcan.service
    - enable: true
    - reload: true
    - watch:
      - file: slcan_interface_service

uavcan:
  git.latest:
    - name: https://github.com/UAVCAN/libuavcan.git
    - rev: master
    - target: /usr/src/uavcan
    - force_checkout: true
    - require:
      - pkg: git

  cmd.run:
    - cwd: /usr/src/uavcan
    - name: |
        git submodule update --init
        mkdir -p build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
        make
        make install
    - creates: /usr/local/bin/uavcan_monitor
    - requires:
      - pkg: build-essential
      - pip: uavcan-python3
