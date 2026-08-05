[10:59:13] Output window cleared
[10:59:13] Graphs cleared
[10:59:13] Sensor Test Error: Couldn't load link driver: [WinError 10048] Only one usage of each socket address (protocol/network address/port) is normally permitted

Traceback (most recent call last):
  File "C:\Users\manoj\AppData\Local\Programs\Python\Python313\Lib\site-packages\cflib\crazyflie\__init__.py", line 235, in open_link
    self.link = cflib.crtp.get_link_driver(
                ~~~~~~~~~~~~~~~~~~~~~~~~~~^
        link_uri, self._link_quality_cb, self._link_error_cb)
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Users\manoj\AppData\Local\Programs\Python\Python313\Lib\site-packages\cflib\crtp\__init__.py", line 99, in get_link_driver
    instance.connect(uri, link_quality_callback, link_error_callback)
    ~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Users\manoj\AppData\Local\Programs\Python\Python313\Lib\site-packages\cflib\crtp\udpdriver.py", line 68, in connect
    self.socket.bind(('', 2399))
    ~~~~~~~~~~~~~~~~^^^^^^^^^^^^
OSError: [WinError 10048] Only one usage of each socket address (protocol/network address/port) is normally permitted

