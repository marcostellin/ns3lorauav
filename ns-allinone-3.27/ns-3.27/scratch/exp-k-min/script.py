import sys
import configparser
import csv
import os
import shutil
from subprocess import call
import numpy as np

def buildArgs (args):

  concat = ''
  for key,value in args.items ():
    concat = concat + "--" + key + "=" + str(value) + " "

  return concat

if __name__ == '__main__':

  config = configparser.ConfigParser ()
  config.read (sys.argv[1])
  time = config.get('settings','time')
  numTeams = config.get('settings','numTeams')
  numMembers = config.get('settings','numMembers')
  #numUavs = config.get('settings','numUavs')
  size = config.get('settings','size')
  runs = config.get('settings','runs')

  filename = 'exp-k-min'
  numUavsList = [6, 8, 10]
  #numUavsList = [5,6]
  
  for numUavs in numUavsList:

    for k in np.arange (0.5, 5, 0.5):
      os.makedirs ('./stats/') 
      for run in range(int(runs)):
        arguments = {'Time':time, 'NumTeams':numTeams, 'NumUAVs':numUavs, 'NumMembers':numMembers, 'Size':size, 'Run':run, 'Kata':k}
        command = "../../waf"
        arg = buildArgs (arguments)
        tot = "\"" + filename + " " + arg + "\""
        print(tot)
        ns3call = call(command + " --run " + tot, shell="True")

      #Parse file and compute stats
      with open('./stats/lora-stats-avg.out.txt', mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        psr = 0
        delayGw = 0
        delayBs = 0
        nRows = 0
        for row in reader:
          nRows = nRows + 1
          psr += float(row['psr'])
          delayGw += float(row['delayGw'])
          delayBs += float(row['delayBs'])

        fname = './results/lora-stats-avg.out.txt'
        exist = False
        if os.path.isfile(fname):
          exist = True

        file = open (fname, 'a+')

        if not exist:
          file.write ('numUavs,k,psr,delayGw,delayBs\n')

        file.write (str(numUavs) + ',' + str(k) + ',' + str(psr/nRows) + ',' + str(delayGw / nRows) + ',' + str(delayBs / nRows) + '\n')

        file.close ()

      with open('./stats/mesh-pdr-stats.out.txt', mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        psr = 0
        nRows = 0
        for row in reader:
          nRows += 1
          psr += float(row['psr'])

        fname = './results/mesh-pdr-stats.out.txt'
        exist = False
        if os.path.isfile(fname):
          exist = True

        file = open (fname, 'a+')

        if not exist:
          file.write ('numUavs,k,psr\n')

        file.write (str(numUavs) + ',' + str(k) + ',' + str(psr/nRows) + '\n')

        file.close ()

      with open ('./stats/coverage-avg-stats.out.txt', mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        covered = 0
        nRows = 0
        for row in reader:
          nRows += 1
          covered += float(row['covered'])

        fname = './results/coverage-avg-stats.out.txt'
        exist = False
        if os.path.isfile(fname):
          exist = True

        file = open (fname, 'a+')

        if not exist:
          file.write ('numUavs,k,covered\n')

        file.write (str(numUavs) + ',' + str(k) + ',' + str(covered/nRows) + '\n')

        file.close ()

      with open ('./stats/bs-disconnection-avg-stats.out.txt', mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        disctime = 0
        nRows = 0
        for row in reader:
          nRows += 1
          disctime += float(row['disctime'])

        fname = './results/bs-disconnection-avg-stats.out.txt'
        exist = False
        if os.path.isfile(fname):
          exist = True

        file = open (fname, 'a+')

        if not exist:
          file.write ('numUavs,k,disctime\n')

        file.write (str(numUavs) + ',' + str(k) + ',' + str(disctime/nRows) + '\n')

        file.close ()

      shutil.rmtree('./stats/')


    
