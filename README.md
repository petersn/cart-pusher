cart-pusher
===========

Cart pusher is a simple team PvE FPS.
Under Debian-like the following should suffice for deps:

    $ sudo apt-get install python-numpy libsdl1.2-dev libsdl-mixer1.2-dev libbullet-dev

To run:

    $ make
    $ python server.py
    ...
    $ python client.py --host server_host --name desired_handle # <-- Each client runs this.

