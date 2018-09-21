hostapd:
  pkg.installed: []
  file.managed:
    - name: /etc/hostapd/hostapd.conf
    - source: salt://wifi/hostapd.conf

hostapd_service:
  file.managed:
    - name: /etc/default/hostapd
    - source: salt://wifi/hostapd_defaults
  service.running:
    - name: hostapd
    - enable: true
    - unmask: true
    - require:
      - pkg: hostapd
      - file: hostapd
      - file: hostapd_service
    - watch:
      - file: hostapd
