# run.ps1 - Copy lane_follow.py to the car and run it, all in one step.
#
# Use it from the repo folder, in PowerShell, with the laptop on the car's Wi-Fi:
#   powershell -ExecutionPolicy Bypass -File .\run.ps1 --camtest
#   powershell -ExecutionPolicy Bypass -File .\run.ps1 --dry
#   powershell -ExecutionPolicy Bypass -File .\run.ps1 --time 60
# Everything you type after run.ps1 is passed to lane_follow.py as options.
#
# Change the IP below to the car you are using. It is shown on the car's screen
# and it can change after the car is restarted.
#   Car 6 (qcar-67815) = 192.168.2.183   (current setting)
#   Car 5 = 192.168.2.217
$CarIP = "192.168.2.183"

# Step 1: copy the newest script to the car (the password is: nvidia).
scp src\lane_follow.py nvidia@${CarIP}:~/

# Step 2: log in to the car and run the script. Ctrl+C still stops the car.
ssh -t nvidia@${CarIP} "PYTHONPATH=~/Documents/Quanser/libraries/python python3 lane_follow.py $args"
