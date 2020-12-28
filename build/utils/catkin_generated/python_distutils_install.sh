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

echo_and_run cd "/home/harshil/niyantum/src/utils"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/harshil/niyantum/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/harshil/niyantum/install/lib/python2.7/dist-packages:/home/harshil/niyantum/build/utils/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/harshil/niyantum/build/utils" \
    "/usr/bin/python2" \
    "/home/harshil/niyantum/src/utils/setup.py" \
     \
    build --build-base "/home/harshil/niyantum/build/utils" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/harshil/niyantum/install" --install-scripts="/home/harshil/niyantum/install/bin"
