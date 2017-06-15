sudo apt-get update

## MOOS-IvP
cd ~
svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk/ moos-ivp
cd ~/moos-ivp
svn update

sudo apt-get install  g++  cmake  xterm
sudo apt-get install  libfltk1.3-dev  freeglut3-dev  libpng12-dev  libjpeg-dev
sudo apt-get install  libxft-dev  libxinerama-dev   libtiff5-dev

./build-moos.sh
./build-ivp.sh

## Python BOOST
sudo apt-get install build-essential g++ python-dev autotools-dev libicu-dev build-essential libbz2-dev
cd ~
wget -O boost_1_63_0.tar.gz http://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.tar.gz/download
tar xzvf boost_1_63_0.tar.gz
cd ~/boost_1_63_0/
./bootstrap.sh --prefix=/usr/local
sudo ./b2 install

## Pymoos
mkdir ~/python-moos/
cd ~/python-moos
git clone https://github.com/themoos/python-moos.git src
mkdir build
cd build
cmake ../src
make
mv lib/pymoos.so ../../moos-ivp/moos-ivp-reed/src/Python/

# GEOS
wget http://download.osgeo.org/geos/geos-3.6.1.tar.bz2
tar xjf geos*bz2
cd geos*
./configure --enable-python
make -j 10
sudo make install

## GDAL
cd ~
sudo apt-get install libproj-dev libgeos-dev
svn checkout https://svn.osgeo.org/gdal/trunk/gdal gdal
cd gdal
./configure --with-python -with-geos=yes --prefix=/usr/local/gdal
make
sudo make install

# Editing .bashrc for GDAL
echo '# GDAL' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/gdal/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PATH=$PATH:/usr/local/gdal/bin' >> ~/.bashrc
echo 'export PYTHONPATH=/usr/local/gdal/lib/python2.7/site-packages/:$PYTHONPATH' >> ~/.bashrc

## Sam Reed's MOOS-IvP-Extend
cd ~/moos-ivp
git clone https://github.com/sji367/moos-ivp-reed.git
./build.sh

# Editing .bashrc for MOOS-IvP
echo '# MOOS' >> ~/.bashrc
echo 'export PATH=$PATH:~/moos-ivp/ivp/bin:~/moos-ivp/ivp/MOOS/MOOSBin' >> ~/.bashrc
echo 'export PATH=$PATH:~/moos-ivp/moos-ivp-reed/bin' >> ~/.bashrc
echo 'export IVP_BEHAVIOR_DIRS=~/moos-ivp/moos-ivp-reed/lib' >> ~/.bashrc

source ~/.bashrc

# Adding python libraries
sudo apt install python-pip
pip install --upgrade pip
sudo pip install pyproj
sudo pip install pytides



