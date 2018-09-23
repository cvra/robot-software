# Prevent the system from hanging at boot until a network comes up
systemd-networkd-wait-online.service:
  service.masked

wlan_conf:
  file.managed:
    - name: /etc/netplan/01-wlan.yaml
    - source: salt://wifi/wlan_config.yaml
  cmd.run:
    - name: netplan apply
    - wait:
      - file: wlan_conf

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

dnsmasq:
  pkg.installed: []
  file.managed:
    - name: /etc/dnsmasq.conf
    - source: salt://wifi/dnsmasq.conf
  service.running:
    - watch:
      - file: dnsmasq

