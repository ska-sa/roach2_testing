#!/usr/bin/env python

import corr, time, struct, sys, logging, socket
from corr import katcp_wrapper
import construct

# Command line options
boffile = '1.bof'
testDuration = 1
reportLen = 10


errors = [0,0,0,0]

qdr_fail = [0,0,0,0]

num_qdr = 4

# Tests to be performed
test_string=['Address','A5','F0','Walking 0s','Walking 1s','Pseudo Random Number']


fpga    = []



def exit_fail():
    print 'FAILURE DETECTED. Log entries:\n',lh.printMessages()
    exit()

def exit_clean():
    try:
        for f in fpga: f.stop()
    except: pass
    exit()
    
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
def exec_test(qdr,test,addr_sel):
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

if __name__ == '__main__':
    from optparse import OptionParser

    p = OptionParser()
    p.set_usage('roach2_qdr_tst.py <ROACH_HOSTNAME_or_IP> [options]')
    p.set_description(__doc__)
    p.add_option('-b', '--boffile', dest='bof', type='str',
        help='Specify the bof file to load')
    p.add_option('-d', '--testduration', dest='td', type='int',
        help='Specify the test duration')
    p.add_option('-r', '--reportlen', dest='rplen', type='int',
        help='Specify the ammount of errors to report') 
    p.add_option('-s', '--TXRXsnapdump', dest='txrxsnapdump', type='int',
        help='Specify the ammount of errors to report') 
    opts, args = p.parse_args(sys.argv[1:])

    if args==[]:
        print 'Please specify a ROACH board. \nExiting.'
        exit()
    else:
        roach = args[0]
   
    if opts.bof != 'None':
       boffile = opts.bof
    if opts.td > 0:
       testDuration = opts.td
    else:
       testDuration = 1 
    if opts.rplen > 0:
       reportLen = opts.rplen
    else:
       reportLen = 4
    if opts.txrxsnapdump > 0:
       txrxsnapdump = opts.txrxsnapdump
    else:
       txrxsnapdump = 0   
    
    
try:
    lh=corr.log_handlers.DebugLogHandler()
    logger = logging.getLogger(roach)
    logger.addHandler(lh)
    logger.setLevel(10)

    print('Connecting to server %s... '%(roach)),
    fpga = corr.katcp_wrapper.FpgaClient(roach, logger=logger)
    time.sleep(1)

    if fpga.is_connected():
        print 'ok\n'
    else:
        print 'ERROR connecting to server %s.\n'%(roach)
        exit_fail()
    
#     if opts.bof != '':
#         print '------------------------'
#         print 'Configuring FPGA...',
#         sys.stdout.flush()
#         fpga.progdev(boffile)
#        time.sleep(5)
#         print 'ok'
#     else:
#         print '------------------------------'
#         print 'No programming file specified!'  
#         print 'Skipping FPGA Configuration...'
#         print '------------------------------'
 
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
#          exit_fail()          
          
       phy_ready=fpga.read_int('phy_rdy{0}'.format(qdr))
       if phy_ready != 1:
          qdr_fail[qdr] = 1
          print ('>>>>>ERR: QDR {0} '.format(qdr)+' is not ready! Skipping tests.')
          time.sleep(1)
#          exit_fail()
                        
    print '---------------------------------'
    print ' Incremental Address Test Suite  '  
    print '---------------------------------'       

    for qdr in range(0, num_qdr):
       if qdr_fail[qdr] == 0:  
          for test in range(0, 6):
             err_cnt_tmp = exec_test(qdr,test,0)
#   Global error count
             errors[qdr] = errors[qdr] + err_cnt_tmp
       
    print '---------------------------------'
    print ' Random Address Test Suite       '  
    print '---------------------------------'
     
    for qdr in range(0, num_qdr): 
       if qdr_fail[qdr] == 0: 
          for test in range(0, 6):
             err_cnt_tmp = exec_test(qdr,test,16)
#   Global error count
             errors[qdr] = errors[qdr] + err_cnt_tmp  
          
    for qdr in range(0, num_qdr):
       if qdr_fail[qdr] == 0:                  
          print 'Summary QDR {0}: '.format(qdr)+'Total Test Errors:  {0}'.format(errors[qdr]) 
       else:
          print 'Summary QDR {0}: '.format(qdr)+'No tests performed due to calibration failure!'
    print '---------------------------------'
    print ' Test Suite Done                 '  
    print '---------------------------------'      
       
except KeyboardInterrupt:
    exit_clean()
except:
    exit_fail()

exit_clean()


