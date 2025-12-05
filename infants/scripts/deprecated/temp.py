import paramiko

hostname = "192.168.253.101"
username = "ut austin"
password = "1234"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(hostname, username=username, password=password)

stdin, stdout, stderr = client.exec_command('w32tm /stripchart /computer:192.168.253.201 /samples:6 /dataonly')
output = stdout.read().decode().strip()
values = re.findall(r'[+-]?\d+\.\d+s', output)
offsets = [float(v.rstrip('s')) for v in values]
mean_offset = sum(offsets[1:]) / len(offsets[1:]) if offsets else None

client.close()

print("Output from Windows machine:\n", output)