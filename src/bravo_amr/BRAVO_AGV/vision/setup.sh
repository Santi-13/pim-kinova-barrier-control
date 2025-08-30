sudo bash ./vision/pyorbbecsdk/scripts/install_udev_rules.sh
export PYTHONPATH=$PYTHONPATH:$(pwd)/vision/pyorbbecsdk/install/lib/
sudo udevadm control --reload-rules && sudo udevadm trigger
