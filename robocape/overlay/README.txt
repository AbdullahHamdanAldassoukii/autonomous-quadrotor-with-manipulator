To install the cape overlays for the UM Robotics Breakout Cape V2

run in this directory:

	$> sudo ./install.sh

then edit /boot/uEnv.txt as root and 

	1. disable HDMI by setting: 
		dtb=am335x-boneblack-emmc-overlay.dtb
	2. disable cape_universal by setting: 
		cmdline=coherent_pool=1M quiet cape_universal=disable
	3. enable the custom olverlays by setting: 
		cape_enable=bone_capemgr.enable_partno=BB-ADC,BB-PWMS,bone_eqep2b,bone_eqep1


finally reboot

