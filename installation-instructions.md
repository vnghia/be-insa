sudo apt-get install pax tar
git clone https://git.openrobots.org/robots/robotpkg.git
cd robotpkg
git clone git://git.openrobots.org/robots/robotpkg/robotpkg-wip wip
export INSTALL_PREFIX=/usr/local/insa/openrobots
./bootstrap/bootstrap --prefix=$INSTALL_PREFIX 
cd math/pinocchio; make install
cd graphics/collada-dom; make install
cd wip/osg-dae; make install
cd graphics/gepetto-viewer-corba; make install

