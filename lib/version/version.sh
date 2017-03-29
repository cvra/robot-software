#!/bin/sh

# abort on errors
set -e
set -u

if [ $# -lt 1 ]
then
    echo "Usage: $0 destination.c"
    exit 1
fi

echo "// AUTOMATICALLY GENERATED" > $1.new
echo "#include <board.h>" >> $1.new
echo "#include <stdint.h>" >> $1.new
echo "const char * software_version_str = \"$(git rev-parse HEAD)-$(git rev-parse --abbrev-ref HEAD)\";" >> $1.new
echo "const uint32_t software_version_short = 0x$(git rev-parse --short HEAD);" >> $1.new
echo "const char * hardware_version_str = BOARD_NAME;" >> $1.new

# If version.c does not exist, overwrite it
if [ ! -e $1 ]
then
    echo "$1 does not exist, creating it..."
    mv $1.new $1
else
    # If there is a version.c, check if the file should change before
    # overriding it.
    # If we don't do this, make will detect a new file and always relink the
    # project.
    if cmp --quiet $1 $1.new
    then
        echo "No change detected in $1 skipping..."
        rm $1.new
    else
        echo "$1 changed, overwriting it"
        mv $1.new $1
    fi
fi
