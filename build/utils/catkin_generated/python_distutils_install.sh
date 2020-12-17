#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/atharva/controls/niyantum/src/utils"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/atharva/controls/niyantum/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/atharva/controls/niyantum/install/lib/python2.7/dist-packages:/home/atharva/controls/niyantum/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/atharva/controls/niyantum/build" \
    "/usr/bin/python2" \
    "/home/atharva/controls/niyantum/src/utils/setup.py" \
     \
    build --build-base "/home/atharva/controls/niyantum/build/utils" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/atharva/controls/niyantum/install" --install-scripts="/home/atharva/controls/niyantum/install/bin"
