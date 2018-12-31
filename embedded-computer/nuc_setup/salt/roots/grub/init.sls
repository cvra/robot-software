/etc/default/grub:
  file.managed:
    - mode: 644
    - source: salt://grub/default
    - user: root
    - group: root
  cmd.wait:
    - name: update-grub
    - watch:
      - file: /etc/default/grub
