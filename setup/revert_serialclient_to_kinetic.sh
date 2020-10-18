#!/bin/sh

# Update
printf "Reveeting SerialClient.py to kinetic to work with rosserial_arduino...  "
sudo cp kineticSerialClient.py /opt/ros/melodic/lib/python2.7/dist-packages/rosserial_python/kineticSerialClient.py
cd /opt/ros/melodic/lib/python2.7/dist-packages/rosserial_python
printf "Renaming old files just in case they are needed...  "
sudo mv SerialClient.pyc melodicSerialClient.pyc
sudo mv SerialClient.py melodicSerialClient.py
printf "Renaming new file to SerialClient.py...  "
sudo mv kineticSerialClient.py SerialClient.py
printf "Done."