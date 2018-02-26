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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/tahlia/cw1/src/comp313p/comp313p_planner_controller"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/tahlia/cw1/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/tahlia/cw1/install/lib/python2.7/dist-packages:/home/tahlia/cw1/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/tahlia/cw1/build" \
    "/usr/bin/python" \
    "/home/tahlia/cw1/src/comp313p/comp313p_planner_controller/setup.py" \
    build --build-base "/home/tahlia/cw1/build/comp313p/comp313p_planner_controller" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/tahlia/cw1/install" --install-scripts="/home/tahlia/cw1/install/bin"
