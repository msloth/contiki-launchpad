#!/usr/bin/python
#
# ------------------------------------------------------------------------------
#
# Display CPU load on a HPDL-1414 display over serial, using psutil
#     http://code.google.com/p/psutil/
#
# Author: Marcus Linderoth
# ------------------------------------------------------------------------------
#
import os, sys, glob, time
import psutil
import serial

sample_len = 0.5

def main():
  t = 0
  serdevs = glob.glob('/dev/tty.uart*')
  for dev in serdevs:
    # check them for the right one...
    print dev

  lpport = "/dev/tty.uart-67FF41CE96103633"   # hard-coded for now
  lpspeed = 9600
  try:
    launchpad = serial.Serial(lpport, lpspeed, timeout=0)
  except:
    print("Heck, couldn't open serial display device")
    sys.exit(-1)

  launchpad.write("HPDL")
  time.sleep(0.5)
  launchpad.write("1414")
  time.sleep(0.5)
  launchpad.write("CPU-")
  time.sleep(0.5)
  launchpad.write("LOAD")
  time.sleep(0.5)

  while True:
      # get and format cpu load; psutil sleeps itself
      cpuload = psutil.cpu_percent(interval = sample_len)
      if cpuload > 100:
        cpuload = 100
      elif cpuload < 0:
        cpuload = 0
      cpuload = str(int(cpuload))  + " %"

      # pad with spaces so we get 4 characters, right just
      if len(cpuload) < 4:
        cpuload = (4 - (len(cpuload))) * " " + cpuload

      # print on HPDL-1414
      print("-" + cpuload + "-")
      launchpad.write(cpuload)

# ------------------------------------------------------------------------------
if __name__=="__main__":
    main()
#-------------------------------------------------------------------------------
