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

# Populate the package build directory with the source files
cp -a ${SRC}/README ${PKGBUILD}/usr/share/doc/python3-pyufr
cp -a ${SRC}/LICENSE ${PKGBUILD}/usr/share/doc/python3-pyufr

cp -a ${SRC}/pyufr.py ${PKGBUILD}/usr/lib/python3/dist-packages

cp -a ${SRC}/examples ${PKGBUILD}/usr/share/doc/python3-pyufr

# Set the version in the control file
sed -i "s/^Version:.*\$/Version: ${VERSION}/" ${PKGBUILD}/DEBIAN/control

# Build the .deb package
fakeroot dpkg -b ${PKGBUILD} ${PKG}
