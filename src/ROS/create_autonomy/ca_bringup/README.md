# ca_bringup

Bring up the real iRobot Create 2.

## Connecting to the robot remotely

### Robot-side

#### Getting robot IP

In your computer, type:

```sh
$ nmap -sP 192.168.0.0/24
```

Keep in mind that the IP might differ.

Check the IP of the device that says *Raspberry Pi*.

#### Configuring robot

```sh
$ ssh pi@<ROBOT_IP>
```

This process will request you the password.

#### Launching the node

Before launching the node you have to configure the proper environmental variables.

```sh
$ roscd ca_bringup/scripts
$ source robot_network_config.sh
$ roslaunch ca_bringup minimal.launch raspicam_receiver_IP:=<COMPUTER_IP>
```

You can get you computer IP doing: `hostname -I`.

### Remote access side

Configure the ROS network variables:

```sh
$ roscd ca_bringup/scripts
$ source remote_access_config.sh <ROBOT_IP>
```

After this step, you can test the raspicam for example:

```sh
$ roslaunch ca_visual_odometry cam_mapping.launch
$ rqt_image_view
```

Whenever you open a new terminal, you'll have to configure the network variables.
