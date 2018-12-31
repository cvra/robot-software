cvra:
  user.present:
    - fullname: CVRA
    - shell: /bin/bash
    - home: /home/cvra
    - groups:
      - sudo
      - dialout
      - plugdev
      - audio

  ssh_auth.present:
    - name: github
    - user: cvra
    - source: salt://users/cvra.keys
