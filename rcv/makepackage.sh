make clean
make
rm -rf deb/usr
mkdir deb/usr
mkdir deb/usr/bin
cp rcv deb/usr/bin
sudo chmod -R 0755 .
dpkg -b deb rcv.deb
