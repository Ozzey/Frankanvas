# Drawing_Demo

## PAINTING

### Tablet Teleoperation

1. Run MIOS server

```bash
cd /home/airocs/Desktop/Projects/mios
desk=false robot_ip=192.168.3.200 websocket_port=12000 database_name=mios verbosity=info ./mios/bin/mios.sh
```

2. Drawing demo

```bash
cd /home/airocs/Desktop/Projects/Drawing_Demo/Visit/build
./Drawing_Demo --robot-ip 127.0.0.1 --mios-port 12000 --local-ip 192.168.3.12 --start-pose 0
```

3. Wacom

```bash
cd /home/airocs/Desktop/Projects
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
python3 Drawing_Demo/scripts/wacom_udp_bridge.py --device-id 12 --host 127.0.0.1 --port 3333 --rate 60
```

### Autonomous Drawing

Use this when you want to give the robot an image and let the Franka draw it with the pen.

1. Run MIOS

Open a terminal and run:

```bash
cd /home/airocs/Desktop/Projects/mios
desk=false robot_ip=192.168.3.200 websocket_port=12000 database_name=mios verbosity=info ./mios/bin/mios.sh
```

Keep this terminal open. If MIOS is already running, skip this step.

2. Convert the image to end-effector points

```bash
cd /home/airocs/Desktop/Projects/Drawing_Demo
python3 scripts/image_to_end_effector.py --image Images/sheikh_zayed.png --name sheikh_zayed
```

This first creates `Data/sheikh_zayed_greyscale.png`, the contrast-enhanced greyscale image used for tracing. The robot replay file and all visuals are generated from this prepared image.

Change the input image here:

```txt
--image Images/sheikh_zayed.png
```

Change the output name here:

```txt
--name sheikh_zayed
```

3. Run the Franka drawing

Open a new terminal while MIOS is still running, then run:

```bash
cd /home/airocs/Desktop/Projects/Drawing_Demo/Visit/build
./Drawing_Demo --robot-ip 127.0.0.1 --mios-port 12000 --local-ip 192.168.3.12 --start-pose 0 --replay-file sheikh_zayed.txt
```

The robot file must match the `--name` from step 2.

```txt
--name sheikh_zayed  ->  --replay-file sheikh_zayed.txt
```
