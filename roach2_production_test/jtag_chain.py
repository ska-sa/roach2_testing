# jtag_chain.py
import urjtag

class Jtag_chain:
  """Detect all the devices on the FTDI JTAG chain.

  Keyword arguments:
  bsdl_path -- full path to the device bsdl files

  """

  def __init__(self, bsdl_path):
    self.bsdlp = bsdl_path

  def scan_chain(self):
    urc = urjtag.chain()
    urc.bsdl_set_path(self.bsdlp)
    urc.cable("roach-2")
    urc.test_cable()
    #not sure why this is neccesary, but without it the chain does not scan
    urc.set_trst(0)
    urc.reset();
    urc.tap_detect()
    urc.disconnect()


