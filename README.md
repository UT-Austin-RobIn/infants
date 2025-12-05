# Infant Experiments
Linux machine log in info:   
username: `robotlearning2`  
password: `robotlearning2`  

You can open this file in chrome for a better viewing experience.
Point the browser to 
`file:///home/robotlearning2/infants/notes.md` 


## NTP time sync 
The linux machine will serve time to the Windows machine.   
Connect the two via ethernet. 
In Windows Administrator CMD prompt: 
`w32tm /query /status`  
That should report `Leap Indicator: 0` and report the linux machine (192.168.253.201) as the source.  
If not, they aren't synchronized.  You can try: 
`w32tm /resync /force`  and then query status again.   
Should that fail, you can restart w32tm. 
`net stop w32time`  
`net start w32time`  
`w32tm /resync /force`   

## Running Trials
1. Open a terminal on linux machine. 
2. Check if NAS (synology) is mounted:
	Run: `mountpoint -q ~/synology-tuli && echo "Mounted" || echo "Not mounted"`
	If output is "Not mounted":
	        Run: `sudo mount -t nfs 192.168.253.1:/volume1/tuli ~/synology-tuli`
3. `cd ~/infants/`
4. Launch the cameras: `./start_all.sh`   
Verify that they start with the command:   
```
./check_cams.sh
```  
You should see all 6 camera topics (color image raw and aligned depth image raw for cameras L, M, R).   

4. Activate the virtualenv, `source ~/envs/infants/bin/activate` and run the experiment script: 
`python experiment/experiment_driver.py`.
5. It will prompt for `subject ID`, `task name`, and `condition ID`. 
Make sure the subject ID 3 digits. Example (1) write 001 for (2) write 002
Subject ID should be an integer. Task should be in `[bang, slide, hammer]`.    
    Condition numbers: 
    1. Soft Board - Headphones		(low haptics,  low audio )
    2. Soft Board - No Headphones 		(low haptics,  high audio)
    3. Hard Board - Headphones 		(high haptics, low audio )
    4. Hard Board - No Headphones 		(high haptics, high audio)
    5. Wash Board - Headphones 		(high haptics, low audio )
    6. Wash Board - No Headphones 		(high haptics, high audio)
    7. Soft Board and Button - Headphones 	(high haptics, low audio )
    8. Soft Board and Button - No Headphones 	(high haptics, high audio)

6. Press ENTER to stop recording that trial. 
7. Say `[y/n]` to keep trial or delete. 
8. Press `ctrl+c` to interrupt and kill the program. 

# Network Setup
Windows IP: `192.168.253.101`  
Linux IP: `192.168.253.201`   
Synology NAS: `192.168.253.1` 
Web interface for synology: `192.168.253.1:5000`.   
Synology: `robin`, `Robot123`.  
To mount: `sudo mount -t nfs 192.168.253.1:/volume1/tuli ~/synology-tuli/`, ps: robotlearning2    
To rsync data: `rsync -r --info=progress2 data /home/robotlearning2/synology-tuli/`   

## ping windows desktop 
set this IP on windows manually if needed. 
`ping 192.168.253.101`

## visualizing data
`rqt_bag <path to bag>` and open a bagfile with the gui. This will show images but audio will not play properly.   
rqt_image_view first and then rosbag play 
You can replay the audio with 
```
c
roslaunch audio_play play.launch
rosbag play <path to bag>
```

additional commands:
1. rs-enumerate-devices


After recording video:
1. python process_marker_tsv.py
2. python visualize_data_on_image.py