### This files describes the steps to configure the Raspberry PI, from the OS to the ROS installation

**An image of the SD card with all the necessary configurations and the initial ROS nodes is available here : .....**

**The steps for the complete configuration are described below :**

## OS Installation
1. Download Ubuntu 22.04 Server for arm64 : https://ubuntu.com/download/server/arm
2. Flash the image on your SD Card (you can use etcher)
3. Insert the SD card in the Raspberry and Power ON the board

## Keyboard configuration
Change the keyboard layout in /etc/default/keyboard. Then reboot.

## Network configuration - Wifi
This procedure is extracted from "https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#4-boot-ubuntu-server"

1. Open "/etc/netplan/50-cloud-init.yaml" file, and add :

_wifis :
      wlan0 :
        dhcp4 : false
        access-points :
          "YourInternetHotspotSSID" :
            password : "password"
          "IOT":
            password : "aBcDeFgHiJINSA18"_

2. Run "sudo netplan apply"
3. Run "sudo reboot"
4. After reboot, you can see the list of available wifi networks : sudo wpa_cli -i wlan0 list_networks
To select a wifi network : 
sudo wpa_cli -i wlan0 select_network <network_id>

## Updates and tools installations
1. General update :
sudo apt update
sudo apt upgrade

2. SSH tools:
sudo apt install openssh-server
sudo service ssh enable
sudo service ssh start

3. Network configuration tools :
sudo apt install net-tools

4. CAN tools :
sudo apt install can-utils

## Firewall configuration
1. Allow all ssh connections :
sudo ufw allow ssh

2. Eth0 multicast configuration :
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
ifconfig eth0 multicast

## CAN configuration

In "/boot/firmware/config.txt" file, add at the end :

_dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835-overlay_

#### Rajouter Service !!!!
To start the CAN, you just have to execute :
"sudo ip link set can0 up type can bitrate 400000"
######################

## Installation of dhcp server (for ethernet)
This procedure is extracted from "https://www.linuxfordevices.com/tutorials/ubuntu/dhcp-server-on-ubuntu"

sudo apt install isc-dhcp-server
sudo mv /etc/dhcp/dhcpd.conf{,.backup}
sudo nano /etc/dhcp/dhcpd.conf :
	# a simple /etc/dhcp/dhcpd.conf
	default-lease-time 600 ;
	max-lease-time 7200 ;
	authoritative ;
 
	subnet 192.168.1.0 netmask 255.255.255.0 {
	 range 192.168.1.100 192.168.1.200 ;
	#option routers 192.168.1.254 ;
	 option domain-name-servers 192.168.1.1, 192.168.1.2 ;
	#option domain-name "mydomain.example" ;
	}

	host archmachine {
	hardware ethernet e0:91:53:31:af:ab; **Adapt the MAC address according to the configured car**
	fixed address 192.168.1.10 ;
	}


Edit the /etc/default/isc-dhcp-server file with: INTERFACESv4="eth0"

Add fixed @IP for eth0: https://ubuntu.com/server/docs/network-configuration 

sudo nano /etc/netplan/99_config.yaml and write:

network:
  version: 2
  renderer: networkd
  ethernets :
    eth0:
      addresses:
        - 192.168.1.1/24

Then :
sudo netplan apply
sudo reboot

We can ping the jetson card : ping 192.168.1.10
 DHCP configuration---FIN

## Change Login and hostname (by "pi" and "geicar")
This procedure is extracted from "https://www.hepeng.me/changing-username-and-hostname-on-ubuntu/"

**LOGIN :**
Login using your username and password.
Set a password for the "root" account.
sudo passwd root 
Log out.
exit
Log in using the "root" account and the password you set earlier.
Change the username and home folder to the new name you want.
usermod -l <newname> -d /home/<newname> -m <oldname>
Change the group name to the new name you want. I'm not sure which group I should change
groupmod -n <newgroup> <oldgroup> 
Lock down the "root" account.
passwd -l root
Logout.
Exit

**HOSTNAME :**
sudo nano /etc/hostname
Delete the old name and configure the new one.
Then edit the /etc/hosts file:
sudo nano /etc/hosts
Replace all occurrences of the existing computer name with your new name.
Reboot the system for the changes to take effect:
sudo reboot


## Installation of ROS2 Humble
Follow what is indicated on this link:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 
+Turtesim:
sudo apt update
sudo apt install ros-humble-turtlesim
+Rqt packages:
sudo apt update
sudo apt install ~nros-humble-rqt*
+rosdep:
sudo apt install python3-rosdep2
update rosdep

+Initialize rosdep dependencies (in ~/ros2_ws !) :
rosdep install -i --from-path src --rosdistro humble -y
+colcon :
sudo apt install python3-colcon-common-extensions
