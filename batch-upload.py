import re, subprocess, json
import platform

isLinux = platform.system() == 'Linux'
if isLinux:
	device_dictionary = 'uploaddict_linux.cfg'
	searchstr = '\/dev\/ttyACM'
else:
	device_dictionary= 'uploaddict.cfg'
	searchstr = 'COM'
	# ports = [re.search('^COM\d+', l).group() for l in out if "COM" in l[:5]]

outstr = subprocess.check_output('pio device list', shell=True).decode('utf-8')
out = outstr.splitlines()

# ports = [re.search('f^{searchstr}\d+', l).group() for l in out if searchstr.replace('\\','') in l]
ports = [l for l in out if searchstr.replace('\\','') in l]
print(f'Discovered devices: {ports}')

with open(device_dictionary) as f:
	data = f.read()
validdevices = json.loads(data)

poplist = [d for d in validdevices.keys() if d.startswith(';') or validdevices[d] not in ports]
for d in poplist:
	validdevices.pop(d)
print(f'Upload list: {validdevices}\n')

for d in validdevices:
	print(f'Uploading {d} to {validdevices[d]}...')
	subprocess.run(f'pio run -t upload -e {d} --upload-port {validdevices[d]}', shell=True)
	# print(f'pio -t upload -e {d} --upload-port {validdevices[d]}')
	print()