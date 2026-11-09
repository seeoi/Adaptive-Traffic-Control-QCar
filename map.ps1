# map.ps1 - Copy map_drive.py (and lane_follow.py, which it uses) to the car and run map_drive.py.
#
# Use it from the repo folder, in PowerShell, with the laptop on the car's Wi-Fi:
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --graph
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --odotest
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --route 9,13 --dry
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --route 9,13
# Everything you type after map.ps1 is passed to map_drive.py as options.
#
# Change the IP below to the car you are using (it is shown on the car's screen).
#   Car 6 (qcar-67815) = 192.168.2.183   (current setting)
#   Car 5 (qcar-67798) = 192.168.2.217
$CarIP = "192.168.2.183"

# Step 1: copy both files to the car in one go (the password is: nvidia).
scp src\lane_follow.py src\map_drive.py nvidia@${CarIP}:~/

# Step 2: log in to the car and run map_drive.py. Ctrl+C still stops the car.
ssh -t nvidia@${CarIP} "PYTHONPATH=~/Documents/Quanser/libraries/python python3 map_drive.py $args"
