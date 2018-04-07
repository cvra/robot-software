This folder contains the TeX source file for my report on the UWB project.

To build it you will need the following tools:

* Python (tested with 3.5)
* Latex
* A set of python packages, run `pip install -r requirements.txt`

To build it, run the following commands:

1. `./convert_ipython.sh` to generate the figures
2. `latexmk -pdf report.tex`
