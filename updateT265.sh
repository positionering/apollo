sudo rm -r /usr/local/include/librealsense2
sudo rm /usr/local/bin/rs-*
sudo rm /usr/local/lib/libreal*
sudo rm -r librealsense
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
echo 'hid_sensor_custom' | sudo tee -a /etc/modules
mkdir build && cd build
cmake ../
sudo make uninstall && make clean && make -j4 && sudo make install

