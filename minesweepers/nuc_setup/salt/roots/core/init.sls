vim:
  pkgrepo.managed:
    - ppa: jonathonf/vim
  pkg.installed: []

git:
  pkgrepo.managed:
    - ppa: git-core/ppa
  pkg.latest:
    - refresh: True

# Friendly Interactive SHell
fish:
  pkgrepo.managed:
    - ppa: fish-shell/release-2
  pkg.latest:
    - refresh: True

mosh:
  pkg.installed

tmux:
  pkg.installed

silversearcher-ag:
  pkg.installed

build-essential:
  pkg.installed

openssh-server:
  pkg.installed

/etc/sudoers.d/passwordless_sudo:
  file.managed:
    - source: salt://core/passwordless_sudo
    - user: root
    - group: root
    - mode: 440

/etc/update-manager/release-upgrades:
  file.managed:
    - source: salt://core/release-upgrades
