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

echo_and_run cd "/home/divyam/turtlebot/src/turtlebot3/turtlebot3_example"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/divyam/turtlebot/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/divyam/turtlebot/install/lib/python2.7/dist-packages:/home/divyam/turtlebot/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/divyam/turtlebot/build" \
    "/usr/bin/python2" \
    "/home/divyam/turtlebot/src/turtlebot3/turtlebot3_example/setup.py" \
    build --build-base "/home/divyam/turtlebot/build/turtlebot3/turtlebot3_example" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/divyam/turtlebot/install" --install-scripts="/home/divyam/turtlebot/install/bin"
