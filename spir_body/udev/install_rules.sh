#!/bin/sh

# Get list of rules in current directory
echo "Files being copied into /etc/udev/rules.d/ include:"

FILES=./*.rules
for f in $FILES
do
  echo "$f"
done

sudo cp ./*.rules /etc/udev/rules.d/

echo "Restarting udev daemon"
sudo service udev restart

