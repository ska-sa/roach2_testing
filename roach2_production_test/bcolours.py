#bcolours.py

class bcolours:
  HEADER = '\033[35m'
  OKBLUE = '\033[34m'
  OKGREEN = '\033[32m'
  WARNING = '\033[33m'
  FAIL = '\033[31m'
  ENDC = '\033[0m'

  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''
