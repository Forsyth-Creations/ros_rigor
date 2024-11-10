# Run 'make help' in the terminal to see a list of script options

SHELL := /bin/bash

# I want this to build all my targets

.PHONY: all
all: # Builds all the ROS targets
	cd hermes_swerve_module && colcon build
	cd hermes_robot_description && colcon build
	cd hermes_robot && colcon build

	# Make the URDF files for the robot
	# cd hermes_robot_description/urdf && gz sdf -p hermes.urdf > hermes.sdf

	# After the build, generate a script to source the setup.bash files
	@echo "Generating setup_env.sh script"
	echo ". $(PWD)/hermes_swerve_module/install/setup.bash" > setup_env.sh
	echo ". $(PWD)/hermes_robot_description/install/setup.bash" >> setup_env.sh
	echo ". $(PWD)/hermes_robot/install/setup.bash" >> setup_env.sh
	chmod +x setup_env.sh
	@echo "Run 'source setup_env.sh' to source the setup files"

# Optional: If you want to source the script automatically (e.g., for CI or convenience), uncomment the next line
.PHONY: source-env
source-env:
	@echo "Sourcing the setup_env.sh"
	@source ./setup_env.sh

.PHONY: clean
clean: # Cleans all the ROS targets
	cd hermes_swerve_module && rm -rf build install log
	cd hermes_robot_description && rm -rf build install log
	cd hermes_robot && rm -rf build install log
	rm -f setup_env.sh
