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
2. `cd ~/infants/`
3. Launch the cameras: `./start_all.sh`   
Verify that they start with the command:   
```
./check_cams.sh
```  
You should see all 6 camera topics (color image raw and aligned depth image raw for cameras L, M, R).   

4. Activate the virtualenv, `source ~/envs/infants/bin/activate` and run the experiment script: 
`python experiment/experiment_driver.py`.
5. It will prompt for `subject ID`, `task name`, and `condition ID`. 
Subject ID should be an integer. Task should be in `[bang, slide, hammer]`.    
    Condition numbers for banging: 
    1. hard-sphere              (high haptic, high audio)
    2. soft-sphere              (high haptic, low audio)
    3. soft-rattle              (low haptic, high audio)
    4. hard-sphere-muffled      (high haptic, low audio)

    Condition numbers for sliding: 
    1. washboard-sphere         (high haptic, high audio)
    2. soft-sphere              (high haptic, low audio)
    3. soft-rattle              (low haptic, high audio)
    4. washboard-sphere-muffled (high haptic, low audio)

6. Press ENTER to stop recording that trial. 
7. Say `[y/n]` to keep trial or delete. 
8. Press `ctrl+c` to interrupt and kill the program. 

# Network Setup
Windows IP: `192.168.253.101`  
Linux IP: `192.168.253.201`   
Synology NAS: `192.168.253.1` 
Web interface for synology: `192.168.253.1:5000`.   
Synology: `robin`, `Robot123`.  
To mount: `sudo mount -t nfs 192.168.253.1:/volume1/tuli ~/synology-tuli/`    
To rsync data: `rsync -r --info=progress2 data /home/robotlearning2/synology-tuli/`   

## ping windows desktop 
set this IP on windows manually if needed. 
`ping 192.168.253.101`

## visualizing data
`rqt_bag <path to bag>` and open a bagfile with the gui. This will show images but audio will not play properly.   
You can replay the audio with 
```
roslaunch audio_play play.launch
rosbag play <path to bag>
```
