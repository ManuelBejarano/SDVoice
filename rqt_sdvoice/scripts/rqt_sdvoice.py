#!/usr/bin/python

import sys

from rqt_sdvoice.sdvoice import Plugin
from rqt_gui.main import Main

plugin = 'rqt_sdvoice' # rqt_sdvoice.py
main = Main(filename=plugin)
print main
sys.exit(main.main(standalone=plugin))
