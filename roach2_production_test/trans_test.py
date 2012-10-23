#!/usr/bin/env python
'''
Tests ROACH2 mezzanine transceivers using SFP+ boards.
\nAuthor: Jason Manley, June 2012.
'''

import corr,time,numpy,struct,sys,logging,pylab,csv
from corr import termcolors

#bitstream = 'r2_trans_test_2012_Jul_06_1249.bof'
#bitstream = 'r2_trans_test_2012_Oct_04_1114.bof'
#bitstream = 'r2_trans_test_2012_Oct_05_1013.bof'
#bitstream = 'r2_trans_test_2012_Oct_05_1336.bof'
#bitstream = 'r2_trans_test_2012_Oct_15_0930.bof'
bitstream = 'r2_trans_test_2012_Oct_19_0911.bof'
katcp_port=7147

#initial packet rate:
rate_n=2
rate_d=2

def exit_fail(lh):
    print 'FAILURE DETECTED. Log entries:\n',lh.printMessages()
    try:
        fpga.stop()
    except: pass
    raise
    exit()

def exit_clean():
    try:
        fp.close()
        fpga.stop()
    except: pass
    exit()

def ByteToHex( byteStr ):
    """
    Convert a byte string to it's hex string representation e.g. for output.
    """
    # Uses list comprehension which is a fractionally faster implementation than
    # the alternative, more readable, implementation below
    #   
    #    hex = []
    #    for aChar in byteStr:
    #        hex.append( "%02X " % ord( aChar ) )
    #
    #    return ''.join( hex ).strip()        
    return ''.join( [ "%02X " % ord( x ) for x in byteStr ] ).strip()

def ip2str(pkt_ip, verbose = False):
    """
    Returns a dot notation IPv4 address given a 32-bit number.
    """
    ip_4 = (pkt_ip & ((2**32) - (2**24))) >> 24
    ip_3 = (pkt_ip & ((2**24) - (2**16))) >> 16
    ip_2 = (pkt_ip & ((2**16) - (2**8)))  >> 8
    ip_1 = (pkt_ip & ((2**8)  - (2**0)))  >> 0
    ipstr = '%i.%i.%i.%i' % (ip_4, ip_3, ip_2, ip_1)
    if verbose:
        print 'IP(%i) decoded to:' % pkt_ip, ipstr
    return ipstr


#START OF MAIN:
def print_report():
    print "\n\n======================="
    print '\tREPORT'
    print "======================="
    for lnk in range(8):
        rx_n=connections[lnk]
        tx_n=lnk
        failed=False
        if rx_n >= 0:
            if link[rx_n]>0:
                print termcolors.colorize('%2i ->%2i ARP OK!'%(tx_n,rx_n),fg='green'),
                if (rx_cnt_ok[rx_n]>0):
                    print termcolors.colorize('%5.1f%% pkts received ok.'%(round(float(rx_cnt_ok[rx_n])/tx_cnt[tx_n]*100,1)),fg='green'),
                else:
                    failed=True
                if (rx_cnt_bad[rx_n]>10) or (rx_wrd_cnt_fail[rx_n]):
                    print termcolors.colorize('Some bad data received! BER: %f.'%(error_rate[lnk]),fg='red'),
                    failed=True
                if (rx_wrd_cnt_missing[rx_n]>5000):
                    print termcolors.colorize('Lost %i packets.'%(rx_wrd_cnt_missing[rx_n]),fg='red'),
                    failed=True
                if failed==True:
                    print termcolors.colorize('Problem with TX lanes on port %i or RX lanes on port %i.'%(tx_n,rx_n),fg='red'),
        else:
            print termcolors.colorize('%2i ->?? link failed! Could not determine where TX of port %i is routed.'%(tx_n,tx_n),fg='red'),
        print ''


def trans_test(roach, desired_rate = 8.0, pkt_size = 1024, skip = False, debug = False, clk_chk = True, auto_test = False):
    """ Test SFP+ Boards connected to a ROACH2 rev2

    Keyword arguments:
    roach -- ROACH2 ip address or host name    
    desired_rate -- Set the datarate in Gbps. Default: 8
    pkt_size -- Set the packet size in bytes. Will be rounded to nearest 8 bytes. Default: 1024
    skip -- Skip initialisation and reprogramming of FPGAs.
    debug -- Print out complete details of each 10GbE core, including ARP tables.
    clk_chk -- Skip the clock check.
    auto_test -- Automatic test; just run the test for a few seconds and then print a summary'
    """

    try:
        loggers = []
        lh=corr.log_handlers.DebugLogHandler()
        logger = logging.getLogger(roach)
        logger.addHandler(lh)
        logger.setLevel(logging.ERROR)

        print('Connecting to server %s on port %i... '%(roach,katcp_port)),
        sys.stdout.flush()

        fpga = corr.katcp_wrapper.FpgaClient(roach, katcp_port, timeout=10,logger=logger)
        time.sleep(1)

        if fpga.is_connected():
            print 'ok'
        else:
            print 'ERROR connecting to server %s on port %i.\n'%(roach,katcp_port)
            exit_fail(lh)

        connections=[float("-inf"),float("-inf"),float("inf"),float("inf"),float("-inf"),float("-inf"),float("-inf"),float("-inf")]

        print '------------------------'
        print '\tProgramming FPGA...',
        sys.stdout.flush()
        if not skip:
            for tx_n in range(8):
                try:
                    fpga.tap_stop('gbe%i'%tx_n)
                except:
                    continue
            fpga.progdev(bitstream)
            print 'done'
            print '\tEstimated FPGA clock rate:',
            sys.stdout.flush()
            if clk_chk:
                est_fpga_clk=fpga.est_brd_clk()
                print ' %iMHz (expecting %iMHz).'%(est_fpga_clk,200),
                if est_fpga_clk > 201 or est_fpga_clk < 199:
                    raise RuntimeError("FPGA clock rate of %i is outside allowable limits (expecting 200MHz). Sysclk failure."%est_fpga_clk)
                else:
                    print 'ok.'
            else:
                print 'Skipped'
        else:
            print 'Skipped.'
        

        if not skip:
            print '\tStarting tap devices on ports 0 through 7...',
            sys.stdout.flush()
            for tx_n in range(8):
                fpga.tap_start(tap_dev=('gbe%i'%tx_n),device='gbe%i'%tx_n,mac=(20015998304256+tx_n),ip=(10<<24)+10+tx_n,port=10000)
            print 'done'

            print "\tWaiting 10s for ARP to complete...",
            sys.stdout.flush()
            time.sleep(10)
            print 'done'

        if debug:
            for rx_n in range(8):
                print '============================================'
                print '\t\tPORT %i'%rx_n
                print '============================================'
                fpga.print_10gbe_core_details('gbe%i'%rx_n,arp=True)

        print '\tGrabbing port statuses...',
        sys.stdout.flush() 
        stat=[]
        for tx_n in range(8):
            stat.append(fpga.get_10gbe_core_details('gbe%i'%tx_n))
        print 'done'

        print '\tChecking RX bonding of link from mezzanine cards...',
        sys.stdout.flush() 
        for tx_n in range(8):
            if not stat[tx_n]['xaui_chan_bond']:
                print termcolors.colorize('\t\tChannel %i lane bonding FAIL! Bad RX links on mezzanine? No mezzanine present? Lane status: %i %i %i %i'%(tx_n,stat[tx_n]['xaui_lane_sync'][0],stat[tx_n]['xaui_lane_sync'][1],stat[tx_n]['xaui_lane_sync'][2],stat[tx_n]['xaui_lane_sync'][3]),fg='red')
                time.sleep(5)
                #raise RuntimeError('Channel %i lane bonding FAIL! Lane status:'%tx_n,stat[tx_n]['xaui_lane_sync'])
        print 'done'

        print '\tChecking tgtap correctly configured cores...',
        sys.stdout.flush() 
        for tx_n in range(8):
            if not ((stat[tx_n]['arp'][tx_n+10] == 20015998304256+tx_n) and (stat[tx_n]['my_ip'] ==(10<<24)+10+tx_n)):
                print 'Channel %i config FAIL!'%tx_n
                fpga.print_10gbe_core_details('gbe%i'%tx_n,arp=True)
        print 'done' 

        print '\tChecking ARP entries for valid cable links...'
        found_so_far = []
        sys.stdout.flush() 
        for rx_n in range(8):
            found_this_one=False
            ip=0
            while ip<255 and not found_this_one:
                ip += 1
                if stat[rx_n]['arp'][ip] != 0xffffffffffff:
    #                print 'Found an IP at %i...'%ip,
                    if (stat[rx_n]['arp'][ip] != (20015998304256+rx_n)) and not (ip in found_so_far):  #check that it's not our address that we've found!
                        found_so_far.append(ip)
                        connections[ip-10]=rx_n
                        found_this_one = True
                        print termcolors.colorize('\t\tPort %i is receiving from IP %s (port %i).'%(rx_n,ip2str(ip-10),ip-10),fg='green')
    #                else:
    #                    print 'This is our IP or is already accounted for. MAC is %i.'%stat[rx_n]['arp'][ip]
            if found_this_one == False: 
                print termcolors.colorize('\t\tERROR: Port %i is not receiving ARP packets from any other port.'%rx_n,fg='red')
                connections[rx_n]=float("-inf")

        reset=False
        packet_words=pkt_size/8
        on_wire_pkt_size=(packet_words*8)+80
        print '\tSetting the packet size to %i bytes (%i 64-bit words)...'%(packet_words*8,packet_words),
        sys.stdout.flush()
        for tx_n,rx_n in enumerate(connections):
            old_size=fpga.read_uint('pkt_size%i'%tx_n)
            if old_size != packet_words:
                fpga.write_int('pkt_size%i'%tx_n,packet_words)
                sys.stdout.flush()
                reset=True
                print tx_n,
        if reset:
            print ''
        else:
            print 'skipped'

        desired_dn=desired_rate*1000/200./64.
        desired_n=packet_words
        desired_d=max(2,(desired_n/desired_dn))
        rate_d=int(desired_d)-1
        rate_n=int(desired_n)
        act_payld_rate=200.*64*rate_n/(rate_d+1)
        act_rate=act_payld_rate*(on_wire_pkt_size)/pkt_size
        print '\tSetting the payload data rate to %6.2f Mbps in both directions on all ports...'%act_payld_rate,
        sys.stdout.flush()
        for tx_n,rx_n in enumerate(connections):
            old_n=fpga.read_uint('pkt_rate%i_n'%tx_n)
            old_d=fpga.read_uint('pkt_rate%i_d'%tx_n)
            if (old_n != rate_n) or (old_d != rate_d):
                fpga.write_int('pkt_rate%i_n'%tx_n,rate_n)
                fpga.write_int('pkt_rate%i_d'%tx_n,rate_d)
                reset=True
                print tx_n,
                sys.stdout.flush()
        if reset:
            print ''
        else:
            print 'skipped'

        print '\tSetting destination IP addresses and ports...',
        sys.stdout.flush()
        for tx_n,rx_n in enumerate(connections):
            if (rx_n<0):
                print termcolors.colorize('\t\tDisabling TX on port %i due to no associated receiver.'%(tx_n),fg='red')
                fpga.write_int('pkt_size%i'%tx_n,0)
            else:
                fpga.write_int('ip%i'%tx_n,(10<<24)+10+rx_n)
                fpga.write_int('port%i'%tx_n,10000)
        print 'done'

        if reset==True:
            print '\tResetting counters...',
            sys.stdout.flush()
            #fpga.write_int('control',4)
            fpga.write_int('control',0) 
            fpga.write_int('control',2) 
            fpga.write_int('control',0) 
            #fpga.write_int('control',4)
            print 'done'

        print '\tStarting transmission...',
        sys.stdout.flush()
        fpga.write_int('control',4)
        print 'done'

        print '\tChecking that packets are being sent...',
        sys.stdout.flush()
        for tx_n,rx_n in enumerate(connections):
            if not (rx_n<0):
                if fpga.read_uint('tx_pkt_cnt%i'%tx_n)<10: 
                    print 'ERROR: Port %i is not sending any data!'%tx_n
                else:
                    print '%i...'%tx_n,
                    sys.stdout.flush()
        print 'done'

        cont=5
        core_stat=range(8)
        link_stat=range(8)
        link=range(8)
        tx_cnt=range(8)
        tx_tib=range(8)
        rx_cnt_ok=range(8)
        rx_cnt_bad=range(8)
        rx_wrd_cnt_fail=range(8)
        rx_wrd_cnt_missing=range(8)
        error_rate=range(8)
        start_time=time.time()
        while(cont>=0):
            for core in range(8):
                core_stat[core]=fpga.get_10gbe_core_details('gbe%i'%core)
                link_stat[core]=fpga.read_uint('status%i'%core)
                link[core]=True
                for i in range(4):
                    if not core_stat[core]['xaui_lane_sync'][i]: link[core]=False
                if not core_stat[core]['xaui_chan_bond']: link[core]=False
                rx_cnt_ok[core] = fpga.read_uint('rx_pkt_cnt_ok%i'%core)
                rx_cnt_bad[core] = fpga.read_uint('rx_pkt_cnt_bad%i'%core)
                rx_wrd_cnt_fail[core] = fpga.read_uint('rx_wrd_cnt_bad%i'%core)
                rx_wrd_cnt_missing[core] = fpga.read_uint('rx_wrd_cnt_missing%i'%core)
            for core in range(8):
                tx_cnt[core]=fpga.read_uint('tx_pkt_cnt%i'%core)
                tx_tib[core]=tx_cnt[core]*(pkt_size+80)/1024/1024/1024.

            for lnk in range(8):
                try:
                    rx_n=connections[lnk]
                    tx_n=lnk
                    if not (rx_n<0):
                        error_rate[lnk]=float(rx_wrd_cnt_fail[rx_n] + rx_wrd_cnt_missing[rx_n] + rx_cnt_bad[rx_n]*packet_words)/float(tx_cnt[tx_n]*packet_words)
                    else:
                        error_rate[lnk]=-1.
                except ZeroDivisionError:
                    #print 'No errors detected on port %i.'%lnk
                    error_rate[lnk]=-1.


            #Move cursor home:
            print '%c[H'%chr(27)
            #clear the screen:
            print '%c[2J'%chr(27)
            print 'Running time: %is.'%(time.time()-start_time)
            print '' 
            print 'Packet payload size            %6i bytes'%(packet_words*8)
            print 'On wire packet size            %6i bytes'%on_wire_pkt_size
            print 'Packet payload efficiency      %7.2f %%'%(float(packet_words*800)/on_wire_pkt_size)
            print 'TX payload rate                %7.2f Mbps'%act_payld_rate
            print 'TX total rate                  %7.2f Mbps'%act_rate
            print '' 
            print 'Mezzanine mapping:'
            print '0: Card0, port 0'
            print '1: Card0, port 1'
            print '2: Card0, port 2'
            print '3: Card0, port 3'
            print '4: Card1, port 0'
            print '5: Card1, port 1'
            print '6: Card1, port 2'
            print '7: Card1, port 3'
            print '' 

            for lnk in range(8):
                tx_n=lnk
                if link_stat[tx_n]&0b1: print termcolors.colorize('ERR: RX overflow on port %i!'%tx_n,fg='red')
                if link_stat[tx_n]&0b10: print termcolors.colorize('ERR: TX overflow on port %i!'%tx_n,fg='red')
                if not link_stat[tx_n]&(1<<5): print termcolors.colorize('ERR: Link down on port %i!'%tx_n,fg='red')

            print 'LINK       TX (GiB)      TX_pkts     RX_pkts_ok  RX_pkts_bad    64b_fails   Dropped_pkts    BER'
            for lnk in range(8):
                rx_n=connections[lnk]
                tx_n=lnk
                if (rx_n>=0):
                    print termcolors.colorize('%2i ->%2i'%(tx_n,rx_n),fg='green'),
                else:
                    print termcolors.colorize('%2i ->??'%(tx_n),fg='red'),
                #print '%12s'%(ip2str(core_stat[tx_n]['my_ip'])),
                print '%12.2f'%(tx_tib[tx_n]),
                print '%12i'%(tx_cnt[tx_n]),
                try:
                    if (link[lnk] and rx_n>=0):
                        print termcolors.colorize('%12i'%(rx_cnt_ok[rx_n]),fg='green' if rx_cnt_ok[rx_n]>0 else 'red'),
                        print termcolors.colorize('%12i'%(rx_cnt_bad[rx_n]),fg='green' if rx_cnt_bad[rx_n]==0 else 'red'),
                        print termcolors.colorize('%12i'%(rx_wrd_cnt_fail[rx_n]),fg='green' if rx_wrd_cnt_fail[rx_n]==0 else 'red'),
                        print termcolors.colorize('%12i'%(rx_wrd_cnt_missing[rx_n]),fg='green' if rx_wrd_cnt_missing[rx_n]==0 else 'red'),
                        print termcolors.colorize('        %12e'%(error_rate[lnk]),fg='green' if error_rate[lnk]<1e-5 else 'red')
                    else:
                        print termcolors.colorize('           Link down or no packets received.',fg='red')
                except:
                    print termcolors.colorize('           Link down or no packets received.',fg='red')
                     
            if auto_test: 
                    cont-=1

            if cont<=0: 
                print_report()
            else:
                time.sleep(1)

    #TODO: output to log file


    except KeyboardInterrupt:
        print_report()
        exit_clean()
    except:
        exit_fail(lh)

    exit_clean()

