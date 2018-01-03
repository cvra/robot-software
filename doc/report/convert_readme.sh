#!/bin/sh

cd $(dirname $0)


pandoc --from markdown --to latex --output user_guide.tex ../../README.md

# Removes "doc/report" path from figures
sed -i s/doc\\/report\\///g user_guide.tex

# Convert links to footnotes
sed -i "s/\\\\href{\\([^}]*\\)}{\\([^}]*\\)}/\2\\\\footnote{\\\\url{\1}}/" user_guide.tex

pandoc --from markdown --to latex --output code_source_org.tex code_source_org.md

# Use non numbered section for code source organization
sed -i "s/section/section*/" code_source_org.tex
