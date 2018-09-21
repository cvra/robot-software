realsense-repo:
  pkgrepo.managed:
    - humanname: Intel Realsense repository
    - name: deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main
    - dist: bionic
    - keyid: C8B3A55A6F3EFCDE
    - keyserver: keyserver.ubuntu.com
    - require_in:
      - pkg: librealsense2-dkms
      - pkg: librealsense2-dev
      - pkg: librealsense2-udev-rules

librealsense2-dkms:
  pkg.installed

librealsense2-dev:
  pkg.installed

librealsense2-udev-rules:
  pkg.installed

