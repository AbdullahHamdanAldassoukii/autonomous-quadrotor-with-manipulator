all:	
	@make -C robocape
	@make -C lcmtypes
	@make -C imu_lcm_example
	@make -C optitrack/common
	@make -C optitrack
	@make -C blocks
	@make -C dynamixel
	@make -C quadrotor

groundstation:
	@make -C java
	@make -C lcmtypes
	@make -C optitrack/common
	@make -C optitrack
	@make -C blocks
	@make -C dynamixel

clean:
	@make -C robocape -s clean
	@make -C lcmtypes -s clean
	@make -C imu_lcm_example -s clean
	@make -C java -s clean
	@make -C optitrack/common -s clean
	@make -C optitrack -s clean
	@make -C blocks -s clean
	@make -C dynamixel -s clean
	@make -C quadrotor -s clean
