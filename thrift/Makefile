all: thrift 

TARBALL = build/thrift-0.7.0.tar.gz
TARBALL_URL = http://mirror.olnevhost.net/pub/apache//thrift/0.7.0/thrift-0.7.0.tar.gz
SOURCE_DIR = build/thrift-0.7.0
MD5SUM_FILE = thrift-0.7.0.tar.gz.md5
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

thrift: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && chmod +x configure &&./configure --without-python --prefix=`pwd`/../.. && make install
	cd $(SOURCE_DIR)/lib/py && python setup.py build && cp -r build/lib*/* ../../../../src/thrift
	touch thrift
clean:
	-rm -rf $(SOURCE_DIR) thrift
wipe: clean
	-rm -rf build
