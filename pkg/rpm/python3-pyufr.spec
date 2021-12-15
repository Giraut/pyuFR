Name:	python3-pyufr
Version:
Release:	0
Summary:	pyuFR - Pure Python communication class for Digital Logic uFR-series NFC readers
License:	GPL-3.0

Requires:	python3 >= 3.7.0b1, python3-pyserial, python3-websocket-client, python3-requests


%description
pyuFR - Pure Python communication class for Digital Logic uFR-series NFC readers



%post
# RPM-based distributions don't implement the generic, version-independent
# /usr/lib/python3/dist-package directory in the site path, so we need this
# nasty %post and %preun fixup trick to install into the correct path without
# making the package distribution-specific - which unfortunately will break the
# package installation when the user upgrades their python version
SITEPKGDIR=$(python3 -c 'import re,sys; print("".join([p for p in sys.path if re.match("^/usr/lib/python3\.[0-9]+/site-packages", p)]))')

if [ "${SITEPKGDIR}" ]; then
  ln -s /usr/lib/python3/dist-packages/pyufr.py ${SITEPKGDIR}
fi



%preun
SITEPKGDIR=$(python3 -c 'import re,sys; print("".join([p for p in sys.path if re.match("^/usr/lib/python3\.[0-9]+/site-packages", p)]))')

if [ "${SITEPKGDIR}" ] && [ -f ${SITEPKGDIR}/pyufr.py ]; then
  rm -f ${SITEPKGDIR}/pyufr.py
fi



%files
%doc /usr/share/doc/python3-pyufr/README
%doc /usr/share/doc/python3-pyufr/LICENSE
%doc /usr/share/doc/python3-pyufr/examples/async_id_beep.py
%doc /usr/share/doc/python3-pyufr/examples/brady_temp_watch.py
%doc /usr/share/doc/python3-pyufr/examples/increase_rxgain.py
%doc /usr/share/doc/python3-pyufr/examples/rf_field_detector.py
%doc /usr/share/doc/python3-pyufr/examples/rf_field_flasher.py
%doc /usr/share/doc/python3-pyufr/examples/rf_field_flash_morse_code.py
%doc /usr/share/doc/python3-pyufr/examples/scan_nano_online_serials.py

/usr/lib/python3/dist-packages/pyufr.py
