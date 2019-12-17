#!/bin/sh

# abort on errors
set -e
set -u

if [ $# -lt 2 ]
then
    echo "Usage: $0 board_name destination.c"
    exit 1
fi

dst=$2
board_name=$1

echo "// AUTOMATICALLY GENERATED" > $dst.new
echo "#include <stdint.h>" >> $dst.new
echo "const char * software_version_str = \"$(git rev-parse HEAD)-$(git rev-parse --abbrev-ref HEAD)\";" >> $dst.new
echo "const uint32_t software_version_short = 0x$(git rev-parse --short HEAD);" >> $dst.new
echo "const char * hardware_version_str = \"$board_name\";" >> $dst.new

# If version.c does not exist, overwrite it
if [ ! -e $dst ]
then
    echo "$dst does not exist, creating it..."
    mv $dst.new $dst
else
    # If there is a version.c, check if the file should change before
    # overriding it.
    # If we don't do this, make will detect a new file and always relink the
    # project.
    if cmp --quiet $dst $dst.new
    then
        echo "No change detected in $dst skipping..."
        rm $dst.new
    else
        echo "$dst changed, overwriting it"
        mv $dst.new $dst
    fi
fi
