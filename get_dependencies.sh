#usr/bin/env sh
WIFI_MAPPING_DIR=$(pwd)/
DEPS_DIR=$(pwd)/deps
mkdir -p $DEPS_DIR

# get the wifi scanning code
cd $DEPS_DIR
if [ ! -d "wifi_scanner" ]; then
    git clone https://github.com/juancamilog/wifi_scanner
fi
cd wifi_scanner
git pull
./build.sh

# get alglib and eigen (for the gaussian process library)
cd $DEPS_DIR
if [ ! -d "alglib" ]; then
    mkdir -p $DEPS_DIR/alglib
    wget -O $DEPS_DIR/alglib.tgz http://www.alglib.net/translator/re/alglib-3.8.0.cpp.tgz
    tar xvzf $DEPS_DIR/alglib.tgz -C $DEPS_DIR/alglib
    # apply patch to alglib
    patch $DEPS_DIR/alglib/cpp/src/optimization.h $DEPS_DIR/../alglib_patches/optimization.h.patch
    patch $DEPS_DIR/alglib/cpp/src/optimization.cpp $DEPS_DIR/../alglib_patches/optimization.cpp.patch
fi
if [ ! -d "eigen" ]; then
    hg clone https://bitbucket.org/eigen/eigen/
fi

# get the gaussian_process regression library
if [ ! -d "gaussian_process" ]; then
    git clone https://github.com/juancamilog/gaussian_process
fi
cd gaussian_process
git pull
# update links to dependencies
rm -f include/alglib include/eigen
ln -s $DEPS_DIR/alglib/cpp include/alglib
ln -s $DEPS_DIR/eigen include/eigen

./build.sh
cd $WIFI_MAPPING_DIR


