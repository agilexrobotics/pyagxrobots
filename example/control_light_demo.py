
#!/usr/bin/env python3
# coding=utf-8

from pyagxrobots import agxrobots

robots=agxrobots.UGV(bustype='socketcan', channel='can0', bitrate=500000) 

robots.EnableCANCtrl()
robots.EnableLightCtrl()
robots.LightFrontMode(1)