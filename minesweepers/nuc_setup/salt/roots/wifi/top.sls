hostapd:
  pkg.installed: []
  file.managed:
    - name: /etc/hostapd/hostapd.conf
    - source: salt://wifi/hostapd.conf
  service.running:
    - enable: true
    - watch:
      - file: hostapd
