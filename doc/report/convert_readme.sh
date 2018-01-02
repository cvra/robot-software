#!/bin/sh

cd $(dirname $0)


pandoc --from markdown --to latex --output user_guide.tex ../../README.md

# Removes "doc/report" path from figures
sed -i s/doc\\/report\\///g user_guide.tex

# Convert links to footnotes
sed -i "s/\\\\href{\\([^}]*\\)}{\\([^}]*\\)}/\2\\\\footnote{\\\\url{\1}}/" user_guide.tex
