nginx:
  pkg.installed: []
  service.running:
    - watch:
      - file: /etc/nginx/sites-available/default

/etc/nginx/sites-available/default:
  file.managed:
    - source: salt://http/server.conf

/var/www/html/index.html:
  file.symlink:
    - target: /home/cvra/robot-software/minesweepers/live_map.html

