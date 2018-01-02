#!/bin/sh
cd $(dirname $0)

jupyter nbconvert --to notebook --execute "models/Simple model.ipynb"
jupyter nbconvert --to notebook --execute "models/UWB only.ipynb"
jupyter nbconvert --to notebook --execute "models/Differential 2 beacons.ipynb"
jupyter nbconvert --to notebook --execute "models/Bandwidth in half duplex nodes.ipynb"
jupyter nbconvert --to notebook --execute "experiments/range_noise.ipynb"
jupyter nbconvert --to notebook --execute "experiments/Madgwick drift.ipynb"

find . -name "*.nbconvert.ipynb" -delete
