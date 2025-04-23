# Infant Experiments
Linux machine log in info:   
username: `robotlearning2`  
password: `robotlearning2`  

You can open this file in chrome for a better viewing experience.
Point the browser to 
`file:///home/robotlearning2/infants/notes.md` 

## Running Trials

Make sure Linux laptop connected to lab PC and time is synchronized.   
See below --- `chronyc tracking` report latency to the desktop (192.168.253.101).  

1. Open a terminal 
2. `cd ~/infants/`
3. Launch the cameras: `./start_all.sh`   
Verify that they start with the command:   
```
rostopic hz /cam_R/color/image_raw /cam_L/color/image_raw /cam_R/aligned_depth_to_color/image_raw /cam_L/aligned_depth_to_color/image_raw
```
4. Run the experiment script: 
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
Windows IP: `ping 192.168.253.101`  
Linux IP: `192.168.253.201`

## set laptop IP 
Once ethernet is plugged in, it should automatically assign the IP.   
`ip addr` should show `192.168.253.201`.   
You can manually set it like so:   
`sudo ip addr add 192.168.253.201/24 dev enp1s0`

## ping windows desktop 
set this IP on windows manually. 
`ping 192.168.253.101`

## chrony configuration -- add the server to the config and verify 
`sudo vim /etc/chrony/chrony.conf `  
`chronyc sources` # <-- should report 192.168.253.101 with *   
`chronyc tracking` # <-- should tell latency to the server  

