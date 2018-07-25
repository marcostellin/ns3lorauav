import matplotlib.pyplot as plt
import numpy as np
import csv

filename = './results/lora-stats-avg.out.txt'

x = []
psrLora = []
disctime = []
psrMesh = []
covered = []


with open('./results/lora-stats-avg.out.txt', mode='r') as csv_file:
  reader = csv.DictReader(csv_file)
  for row in reader:
    x.append (int(row['numUavs']))
    psrLora.append (float(row['psr']))

with open('./results/coverage-avg-stats.out.txt', mode='r') as csv_file:
  reader = csv.DictReader(csv_file)
  for row in reader:
    covered.append (float(row['covered']))

with open('./results/bs-disconnection-avg-stats.out.txt', mode='r') as csv_file:
  reader = csv.DictReader(csv_file)
  for row in reader:
    disctime.append (float(row['disctime']))

with open('./results/mesh-pdr-stats.out.txt', mode='r') as csv_file:
  reader = csv.DictReader(csv_file)
  for row in reader:
    psrMesh.append (float(row['psr']))

psrTot = np.multiply(psrLora, psrMesh)

plt.plot(x, psrLora, label='PSR LoRaWAN', marker='o')
plt.plot(x, psrMesh, label= 'PSR Mesh', marker='x')
plt.plot(x, disctime, label = 'Disconnection fraction', marker='x')
plt.plot(x, covered, label='Covered Fraction', marker='o')
plt.plot(x, psrTot, marker='o')
#plt.legend()
plt.xlabel('numUavs')
plt.axis([6,10,0,1])
plt.grid(True)
plt.show()