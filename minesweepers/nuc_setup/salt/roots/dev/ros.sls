ros_repo:
  pkgrepo.managed:
    - name: deb http://packages.ros.org/ros/ubuntu bionic main
    - dist: bionic
    - file: /etc/apt/sources.list.d/ros-latest.list
    - key_url: https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

  cmd.run:
    - name: 'apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116'
    - unless: 'apt-key list | grep ros.org'

{% for pkg in ["ros-melodic-ros-base",
         "python-rosinstall",
         "python-catkin-tools",
         "ros-melodic-controller-manager",
         "ros-melodic-diff-drive-controller",
         "ros-melodic-perception",
         "ros-melodic-diagnostic-updater",
] %}
{{ pkg }}:
  pkg.installed:
    - require:
      - pkgrepo: ros_repo
{% endfor %}

rospkg-python3:
  pip.installed:
    - name: rospkg
    - bin_env: /usr/bin/pip3

catkin-pkg-python3:
  pip.installed:
    - name: catkin-pkg
    - bin_env: /usr/bin/pip3

rosdep_init:
  cmd.run:
    - name: "rosdep init"
    - user: root
    - creates: "/etc/ros/rosdep/sources.list.d/20-default.list"

rosdep_update:
  cmd.run:
    - name: "rosdep update"
    - runas: cvra
    - creates: "~/.ros/rosdep/"

ros_source_setup:
  file.append:
    - name: "/etc/bash.bashrc"
    - text:
      - "source /opt/ros/melodic/setup.bash"
      - "source ~/catkin_ws/devel/setup.bash"

catkin_workspace:
  file.directory:
    - name: "/home/cvra/catkin_ws/src"
    - user: cvra
    - group: cvra
    - makedirs: True

  cmd.run:
    - name: "source /opt/ros/melodic/setup.bash; catkin_init_workspace"
    - runas: cvra
    - cwd: "/home/cvra/catkin_ws/src"
    - creates: "/home/cvra/catkin_ws/src/CMakeLists.txt"
    - require:
      - file: catkin_workspace
