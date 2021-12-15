#!/bin/sh

# Directories and files
BUILDSCRIPTPATH=$(realpath "$0")
BUILDSCRIPTDIR=$(dirname ${BUILDSCRIPTPATH})
SRC=$(realpath ${BUILDSCRIPTDIR}/../..)
VERSION=$(grep -E "^ +v[0-9]+\.[0-9]+\.[0-9]+ *$" ${SRC}/README | sed -E 's/[ v]*//')
PKGSPEC=${BUILDSCRIPTDIR}/python3-pyufr.spec
PKG=python3-pyufr-${VERSION}-0.noarch
PKGBUILD=${BUILDSCRIPTDIR}/${PKG}
BUILDROOT=${PKGBUILD}/BUILDROOT/${PKG}
RPMDIR=${PKGBUILD}/RPMS

# Create a fresh RPM build directory
rm -rf ${PKGBUILD}
mkdir -p ${PKGBUILD}/SPECS
mkdir -p ${PKGBUILD}/SOURCES
mkdir -p ${PKGBUILD}/BUILD
mkdir -p ${PKGBUILD}/BUILDROOT
mkdir -p ${PKGBUILD}/RPMS
mkdir -p ${PKGBUILD}/SRPMS

# Copy the spec file into the RPM build directory
cp -a ${PKGSPEC} ${PKGBUILD}/SPECS

# Create empty directory structure
mkdir -p ${BUILDROOT}/usr/lib/python3/dist-packages
mkdir -p ${BUILDROOT}/usr/share/doc/python3-pyufr/examples

# Populate the package build directory with the source files
echo
echo ${BUILDROOT}/usr/share/doc/python3-pyufr
echo
cp -a ${SRC}/README ${BUILDROOT}/usr/share/doc/python3-pyufr
cp -a ${SRC}/LICENSE ${BUILDROOT}/usr/share/doc/python3-pyufr

cp -a ${SRC}/pyufr.py ${BUILDROOT}/usr/lib/python3/dist-packages

cp -a ${SRC}/examples ${BUILDROOT}/usr/share/doc/python3-pyufr

# Set the version in the spec file
sed -i "s/^Version:.*\$/Version: ${VERSION}/" ${PKGBUILD}/SPECS/python3-pyufr.spec

# Build the .rpm package
rpmbuild --target=noarch --define "_topdir ${PKGBUILD}" --define "_rpmdir ${RPMDIR}" -bb ${PKGBUILD}/SPECS/python3-pyufr.spec

# Retrieve the built .rpm package
cp ${RPMDIR}/noarch/${PKG}.rpm ${BUILDSCRIPTDIR}
