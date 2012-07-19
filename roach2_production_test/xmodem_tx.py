# xmodem_tx.py
import os, logging, sys
import xmodem 

class Xmodem_tx: 
  """Send files via the XMODEM protocol on a serial port.

  Keyword arguments:
  serial_obj -- Serial instance, 115200 bps, timeout = 1
  tx_file -- full path of file to transmit

  """

  def __init__(self, serial_obj, tx_file, log_handler):
    self.so  = serial_obj
    self.txf = tx_file
    self.fs = os.path.getsize(tx_file)/1024
    self.blk_cnt = 0
    self.lh = log_handler
    #self.loghandler = self.logger.getHandler('mytx')

  def getc(self, size, timeout=0):
    data = self.so.read(size)
    if data == '':
      return None
    else: 
      if data == '\x06':
        self.blk_cnt = self.blk_cnt + 1
        if (self.blk_cnt % 8) == 0:
          print ('%d of %d kb sent.\r' %(self.blk_cnt*128/1024, self.fs)),
          sys.stdout.flush()
      return data

  def putc(self, data, timeout=0):
    size = self.so.write(data)
    if size == 0:
      return None
    else:
      return size
  
  def xmdm_send(self):
    stream = open(self.txf, 'rb')
    xmodem.log.addHandler(self.lh)
    xmdm = xmodem.XMODEM(self.getc, self.putc)
    result = xmdm.send(stream, retry=30, quiet = 1)
    stream.close()
    return result

   
