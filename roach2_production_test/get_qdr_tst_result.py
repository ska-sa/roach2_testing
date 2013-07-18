#!/usr/bin/python
import sys, argparse
import defs_r2_ats as defs

if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description='Extract ROACH2 QDR test information from log file.')
  parser.add_argument('-m', '--manufacturer', dest='manuf', choices=defs.MANUF, default=defs.MANUF_DEF)
  parser.add_argument('-r', '--revision', dest='rev', type=int, choices=[1,2], default=2, help='ROACH2 revision')
  parser.add_argument('-b', '--batch', dest='batch', type=int, choices=range(0,256))
  parser.add_argument('-o', '--board', dest='board', type=int, choices=range(0,256))
  args = parser.parse_args() 

  manuf = defs.MANUF[args.manuf]
  roach = '01'

  logfile = file('log/qdr_test.log')

  str1 = 'Full QDR test summary'
  if args.batch == None:
    str2 = '{0:02x}{1}{2:02x}'.format(manuf, roach, args.rev)
  elif args.board == None:
    str2 = '{0:02x}{1}{2:02x}{3:02x}'.format(manuf, roach, args.rev, args.batch)
  else:
    str2 = '{0:02x}{1}{2:02x}{3:02x}{4:02x}'.format(manuf, roach, args.rev, args.batch, args.board)

  while True:
    line = logfile.readline()
    if (str1 in line) and (str2 in line):
      print line,
      for i in range(0,4):
        print logfile.readline(),
    if not line: break


  logfile.close()
 
 
