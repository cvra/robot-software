python-pip:
  pkg.installed

python3-pip:
  pkg.installed

{% for pkg in [
    "pyyaml",
    "jinja2",
    "progressbar2",
    "pyserial",
    "msgpack-python",
    "numpy",
    "scipy",
    "scikit-learn",
]%}
{{ pkg }}-python3:
  pip.installed:
    - name: {{ pkg }}
    - bin_env: /usr/bin/pip3
    - require:
      - pkg: python3-pip
{% endfor %}
