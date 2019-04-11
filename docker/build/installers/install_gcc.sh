# please refer to this discussion
# https://gist.github.com/ibogun/ec0a4005c25df57a1b9d
apt-get install python-software-properties -y
add-apt-repository ppa:ubuntu-toolchain-r/test -y
apt-get update -y
apt-get install gcc-5 -y
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 50
apt-get install g++-5 -y
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 50
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
update-alternatives --set gcc "/usr/bin/gcc-5"

apt-get clean && rm -rf /var/lib/apt/lists/*
