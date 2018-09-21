cvra:
  user.present:
    - fullname: CVRA
    - shell: /usr/bin/fish
    - home: /home/cvra
    - groups:
      - sudo
      - dialout
      - plugdev

  ssh_auth.present:
    - name: github
    - user: cvra
    - source: salt://users/cvra.keys
