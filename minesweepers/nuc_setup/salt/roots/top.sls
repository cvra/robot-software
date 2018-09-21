base:
  '*':
    - core
    - users.cvra
    - dev.ros
    - dev.python
    - can

  'virtual:physical':
    - match: grain
    - grub
    - wifi
