unzip HSL.zip
cd HSL
# Move nested HSL solver sources up to the expected coinhsl/ directory
mv coinhsl/coinhsl/* coinhsl/ 2>/dev/null || true
rmdir coinhsl/coinhsl 2>/dev/null || true
mkdir build
cd build
../configure --prefix=/usr/local
make -j$(nproc)
make install
# cd ../..
# unzip hsl_ma57.zip
# cd hsl_ma57
# mkdir build
# cd build
# ../configure
# make
# make install
# cd ../..
# unzip hsl_ma86.zip
# cd hsl_ma86
# mkdir build
# cd build
# ../configure
# make
# make install
# cd /usr/local/lib
# ln -s libhsl_ma57.so libhsl.so