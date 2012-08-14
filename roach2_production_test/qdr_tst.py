#!/usr/bin/env python

import corr, time, struct, sys, logging, socket, telnetlib, shutil, os
import construct



# Tests to be performed
test_string = ['Address','A5','F0','Walking 0s','Walking 1s','Pseudo Random Number']

#******************************************************************************************************************
# Test execution for specified:
#
# qdr: 0,1,2,3 (Which QDR must be tested)
# test: 0-5 (Which test to be done)
#       0 - Address Test (Address is written as data) (Dead Address or Shorted Address Pins)
#       1 - A5 - Alternating bit set between adjacent bits and from cycle to cycle (cross-talk)
#           5A5A 5A5A then A5A5 A5A5 
#       2 - F0 - Alternating bus set between from cycle to cycle (maximum current swing)
#           5A5A 5A5A then A5A5 A5A5
#       3 - Walking '0' - Feedback shift register: All bits are set to '1' except one that is '0' (stuck bits)
#           FFF...FFFE, F...FFFD, F....FFFB, F...FFF7, F...FFEF, ...... 
#       4 - Walking '1' - Feedback shift register: All bits are set to '0' except one that is '1' (stuck bits)
#           0...0001, 0...0002, 0...0004, 0...0008, 0...0010, ...... 
#       5 - Pseudo Random Number - LFSR is used to generate a PRN sequence (typical average operation)
#
#****************************************************************************************************************** 
def exec_test(fpga, qdr, test, addr_sel, testDuration, reportLen, txrxsnapdump):
    """Test to be executed"""
    result = 0
    print '=============================================================='
    print ' QDR {0} '.format(qdr)+'Test: '+test_string[test]+' running...'  
 
#   eg. QDR 1 Test: Address running...

#   Clear all counters and tests
    fpga.write_int('en',0)    
    fpga.write_int('rst',3)
  
#   Select the test to be performed and deassert reset  
    fpga.write_int('tst_sel',test+addr_sel) # bit [2:0] test, bit[4] - 0 inc address, 1 random address
    fpga.write_int('rst',0)
 
#   Prime snap blocks

#   TX
    fpga.write_int('snap_tx_ctrl',0)
    fpga.write_int('snap_tx_ctrl',1)
    fpga.write_int('snap_addr_ctrl',0)
    fpga.write_int('snap_addr_ctrl',1)
    
#   RX
    fpga.write_int('snap_rx0_ctrl',0)
    fpga.write_int('snap_rx0_ctrl',1)
     
#   Snap Blocks for error address and data
#   snap_err_addr - error address 32 bits, but [18:0] effective
#   snap_err_data - error data 128 bits - 127:64 Written Data, 63:0 Read Data (Faulty)
 
    snap_err_data = 'snap_err_data{0}_ctrl'.format(qdr)
    snap_err_addr = 'snap_err_addr{0}_ctrl'.format(qdr)
    
    fpga.write_int(snap_err_data,0)     
    fpga.write_int(snap_err_data,1)
    
    fpga.write_int(snap_err_addr,0)
    fpga.write_int(snap_err_addr,1)
  
#   Enable test  
    fpga.write_int('en',1)
    
#   Wait for 1 sec (about 25 times through all addresses)
    time.sleep(testDuration)
    
    fpga.write_int('en',0)
    
#   Make sure that the data was written & read out   
    tx_cnt=fpga.read_int('tx_cnt')
    rx_cnt=fpga.read_int('rx_cnt{0}'.format(qdr))

    if tx_cnt == 0:
        print ('>>>>>ERR: Not transmitting anything. This should not happen. Exiting.')
        exit_clean()
    if rx_cnt == 0:
        print ('>>>>>ERR: QDR {0} Not reading anything. This should not happen. Exiting.'.format(qdr))
        exit_clean()

#   Read error counter        
    err_cnt=fpga.read_int('err_cnt{0}'.format(qdr))
    
#   snap_err_addr - TX address 32 bits, but [18:0] effective 
    addr_bram_dmp=fpga.read('snap_addr_bram'.format(qdr),4*reportLen) # read 10 x 32bit words (4 bytes)
#   snap_err_data - TX Written Data, 63:0
    wr_data_bram_dmp=fpga.read('snap_tx_bram'.format(qdr),8*reportLen)# read 10 x 2 x 64bit words (16 bytes)
#   snap_err_data - RX 63:0 Read Data
    rd_data_bram_dmp=fpga.read('snap_rx0_bram'.format(qdr),8*reportLen)# read 10 x 2 x 64bit words (16 bytes)
    
    if txrxsnapdump > 0:    
       print 'Reading TX snap blocks'
    for i in range(0,txrxsnapdump):
       addr_32bit = struct.unpack('>L',addr_bram_dmp[(4*i):(4*i)+4])[0] # >L 32bit unsigned
       write_data_64bit = struct.unpack('>Q',wr_data_bram_dmp[(8*i):(8*i)+8])[0] # >Q 64 bit unsigned
       read_data_64bit = struct.unpack('>Q',rd_data_bram_dmp[(8*i):(8*i)+8])[0] # >Q 64 bit unsigned
       print 'Address: %8X, TX Data: %16X, RX Data: %16X'%(addr_32bit,write_data_64bit,read_data_64bit)

#   snap_err_addr - error address 32 bits, but [18:0] effective 
    addr_bram_dmp=fpga.read('snap_err_addr{0}_bram'.format(qdr),4*reportLen) # read 10 x 32bit words (4 bytes)
#   snap_err_data - error data 128 bits - 127:64 Written Data, 63:0 Read Data (Faulty)
    data_bram_dmp=fpga.read('snap_err_data{0}_bram'.format(qdr),16*reportLen)# read 10 x 2 x 64bit words (16 bytes)
           
#   Generate error report            
    if err_cnt != 0:
        print '>>>>>ERR: QDR {0} '.format(qdr)+test_string[test]+': {0} Test Errors'.format(err_cnt)
        
        print 'Reading error snap blocks'
        for i in range(0,reportLen):
           addr_32bit = struct.unpack('>L',addr_bram_dmp[(4*i):(4*i)+4])[0] # >L 32bit unsigned
           write_data_64bit = struct.unpack('>Q',data_bram_dmp[(16*i):(16*i)+8])[0] # >Q 64 bit unsigned
           err_rd_data_64bit = struct.unpack('>Q',data_bram_dmp[(16*i)+8:(16*i)+16])[0] # >Q 64 bit unsigned
           print 'Address: %8X Expected Data: %16X Received Data: %16X'%(addr_32bit,write_data_64bit,err_rd_data_64bit)
           
    print ' QDR {0} '.format(qdr)+'Test: '+test_string[test]+' completed.'
    print '=============================================================='
    
    result = err_cnt
    return result

def exit_clean():
    try:
        for f in fpgas: f.stop()
    except: pass
    try:
        for t in teln: t.close()
    except: pass

def test_qdr(roachIP, boffile, testDuration = 1, reportLen = 10, txrxsnapdump = 0):
  """Test QDR's

  Keyword arguments:
  roachIP -- ROACH2 ip address string (netbooted)
  boffile -- Boffile to program
  testDuration -- Specifies the test duration
  reportLen -- Specifies the ammount of errors to report
  txrxsnapdump -- Number of snap blocks to display

  """

  errors = [0,0,0,0]
  qdr_fail = [0,0,0,0]
  num_qdr = 4
  fpga = []
      
      
  try:
      lh=corr.log_handlers.DebugLogHandler()
      logger = logging.getLogger('R2_QDR_Testing')
      logger.addHandler(lh)
      logger.setLevel(10)

      print('    Connecting to TCPBorph server on %s... '%(roachIP)),
      sys.stdout.flush()
      fpga = corr.katcp_wrapper.FpgaClient(roachIP, logger = logger)
      time.sleep(1)
      if not fpga.is_connected():
        raise Exception('ERROR: Connection to TCPBorph server not successful.')
      print 'done.'
      print '    Configuring FPGA...',
      sys.stdout.flush()
      try:
        with open('/home/nfs/roach2/current/boffiles/%s'%boffile) as f: pass
      except:
        inpath = 'support_files/%s'%boffile
        outpath = '/home/nfs/roach2/current/boffiles/%s'%boffile
        shutil.copyfile(inpath, outpath)
        os.chmod(outpath, 0777)
      try:
        teln = telnetlib.Telnet(roachIP, 7147)
      except:
        raise
      teln.write('?progdev %s\n'%boffile)
      timeout = 0
      found = False
      response = ''
      while not found and timeout < 10:
        time.sleep(1)
        timeout += 1
        response = response + teln.read_very_eager()
        if response.find('!progdev ok') <> -1:
          found = True
      if not found:
        teln.close()
        raise Exception('ERROR: Boffile not programmed. Error message:\n%s'%response)
      print 'boffile programmed.'
      teln.close()
      sys.stdout.flush()

      print '---------------------------------'
      print ' Checking QDR Calibration Status '  
      print '---------------------------------'
      
#   Check that QDR's are calibrated and ready before doing any tests!   
      for qdr in range(0, num_qdr):       
         cal_fail=fpga.read_int('cal_fail{0}'.format(qdr))
         if cal_fail != 0:
            qdr_fail[qdr] = 1     
            print ('>>>>>ERR: QDR {0} '.format(qdr)+' did NOT calibrate! Skipping tests.')
            time.sleep(1)          
            
         phy_ready=fpga.read_int('phy_rdy{0}'.format(qdr))
         if phy_ready != 1:
            qdr_fail[qdr] = 1
            print ('>>>>>ERR: QDR {0} '.format(qdr)+' is not ready! Skipping tests.')
            time.sleep(1)
                          
      print '---------------------------------'
      print ' Incremental Address Test Suite  '  
      print '---------------------------------'       

      for qdr in range(0, num_qdr):
         if qdr_fail[qdr] == 0:  
            for test in range(0, 6):
               err_cnt_tmp = exec_test(fpga, qdr, test, 0, testDuration, reportLen, txrxsnapdump)
               #   Global error count
               errors[qdr] = errors[qdr] + err_cnt_tmp
         
      print '---------------------------------'
      print ' Random Address Test Suite       '  
      print '---------------------------------'
       
      for qdr in range(0, num_qdr): 
         if qdr_fail[qdr] == 0: 
            for test in range(0, 6):
               err_cnt_tmp = exec_test(fpga, qdr, test, 16, testDuration, reportLen, txrxsnapdump)
               #   Global error count
               errors[qdr] = errors[qdr] + err_cnt_tmp  

      print ''
      print '---------------------------------'
      print ' Test Summary                    '  
      print '---------------------------------'
            
      for qdr in range(0, num_qdr):
         if qdr_fail[qdr] == 0:                  
            print 'QDR {0}: '.format(qdr)+'Total Test Errors:  {0}'.format(errors[qdr]) 
         else:
            print 'QDR {0}: '.format(qdr)+'No tests performed due to calibration failure!'
         
  except KeyboardInterrupt:
      exit_clean()
  except:
      #exc_type = sys.exc_info()[0]
      #exc_mess = sys.exc_info()[1]
      #print 'Exception type: %s Message: %s' %(exc_type, exc_mess)
      exit_clean()
      raise

  exit_clean()
  # Add qdr_fail and error lists together to determine if any errors occured.
  if (reduce(lambda x,y: x+y, qdr_fail) + reduce(lambda x,y: x+y, errors)):
    return False
  else:
    return True


