#!/bin/sh

# Directories and files
BUILDSCRIPTPATH=$(realpath "$0")
BUILDSCRIPTDIR=$(dirname ${BUILDSCRIPTPATH})
SRC=$(realpath ${BUILDSCRIPTDIR}/../..)
PKGSRC=${BUILDSCRIPTDIR}/python3-pyufr
VERSION=$(grep -E "^ +v[0-9]+\.[0-9]+\.[0-9]+ *$" ${SRC}/README | sed -E 's/[ v]*//')
PKGBUILD=${PKGSRC}-${VERSION}-0_all
PKG=${PKGBUILD}.deb

# Create a fresh skeleton package build directory
rm -rf ${PKGBUILD}
cp -a ${PKGSRC} ${PKGBUILD}

# Create empty directory structure
mkdir -p ${PKGBUILD}/usr/lib/python3/dist-packages
mkdir -p ${PKGBUILD}/usr/share/doc/python3-pyufr/examples

# Populate the package build directory with the source files
install -m 644 ${SRC}/README ${PKGBUILD}/usr/share/doc/python3-pyufr
install -m 644 ${SRC}/LICENSE ${PKGBUILD}/usr/share/doc/python3-pyufr

install -m 644 ${SRC}/pyufr.py ${PKGBUILD}/usr/lib/python3/dist-packages

install -m 755 ${SRC}/examples/* ${PKGBUILD}/usr/share/doc/python3-pyufr/examples

# Set the version in the control file
sed -i "s/^Version:.*\$/Version: ${VERSION}/" ${PKGBUILD}/DEBIAN/control

# Fixup permissions
find ${PKGBUILD} -type d -exec chmod 755 {} \;
chmod 644 ${PKGBUILD}/DEBIAN/control
chmod 644 ${PKGBUILD}/usr/share/doc/python3-pyufr/copyright

# Build the .deb package
fakeroot dpkg -b ${PKGBUILD} ${PKG}
