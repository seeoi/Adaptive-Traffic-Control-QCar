# map.ps1 - Copy map_drive.py (and lane_follow.py, which it uses) to the car and run map_drive.py.
#
# Use it from the repo folder, in PowerShell, with the laptop on the car's Wi-Fi:
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --graph
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --odotest
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --route 9,13 --dry
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --route 9,13
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --gpscal      (LiDAR room scan, car at x=0 y=2 facing down)
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --whereami    (motor OFF, prints the LiDAR position)
#   powershell -ExecutionPolicy Bypass -File .\map.ps1 --route 22,9,13,19,17,20,22 --gps
# Everything you type after map.ps1 is passed to map_drive.py as options.
#
# Change the IP below to the car you are using (it is shown on the car's screen).
#   Car 6 (qcar-67815) = 192.168.2.183
#   Car 7              = 192.168.2.45    (current setting)
#   Car 5 (qcar-67798) = 192.168.2.217
# Another car without editing this file: in PowerShell first type   $env:QCAR_IP = "192.168.2.183"
$CarIP = if ($env:QCAR_IP) { $env:QCAR_IP } else { "192.168.2.45" }

# Step 1: copy both files to the car in one go (the password is: nvidia).
scp src\lane_follow.py src\map_drive.py nvidia@${CarIP}:~/

# Step 2: log in to the car and run map_drive.py. Ctrl+C still stops the car.
ssh -t nvidia@${CarIP} "PYTHONPATH=~/Documents/Quanser/libraries/python python3 map_drive.py $args"
