# Run 'make help' in the terminal to see a list of script options

SHELL := /bin/bash

# I want this to build all my targets

.PHONY: all
all: # Builds all the ROS targets
	$(MAKE) reset_setup
	# call the no-web

	cd /ros2_jazzy/workspace/web_controller/web_controller/backend && poetry config virtualenvs.create false && poetry install
	cd /ros2_jazzy/workspace/web_controller/web_controller/frontend && npm install -g && npm install && npm run build
	
	# Delete the node_modules folder
	rm -rf web_controller/install/web_controller/share/web_controller/frontend/node_modules

	cd web_controller && colcon build

	# Go into the share and install the modules. A hack, but it works
	cd web_controller/install/web_controller/share/web_controller/frontend && npm install

	$(MAKE) no-web
	chmod +x setup_env.sh
	@echo "Run 'source setup_env.sh' to source the setup files"

.PHONY: reset_setup
reset_setup:
	rm -f setup_env.sh

.PHONY: no-web
no-web:
	cd hermes_swerve_module && colcon build
	cd hermes_robot_description && colcon build
	cd hermes_robot && colcon build
	# After the build, generate a script to source the setup.bash files
	@echo "Generating setup_env.sh script"
	echo ". $(PWD)/hermes_swerve_module/install/setup.bash" > setup_env.sh
	echo ". $(PWD)/hermes_robot_description/install/setup.bash" >> setup_env.sh
	echo ". $(PWD)/hermes_robot/install/setup.bash" >> setup_env.sh
	echo ". $(PWD)/web_controller/install/setup.bash" >> setup_env.sh

	make validation


.PHONY: validation
validation:
	cd hermes_validation_nodes && colcon build
	echo ". $(PWD)/hermes_validation_nodes/install/setup.bash" >> setup_env.sh

.PHONY: realsense
realsense:
	cd realsense_gazebo_plugin && colcon build

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
	cd web_controller/web_controller/backend && rm -rf .venv
	cd web_controller && rm -rf build install log
	rm -f setup_env.sh
	rm -rf web_controller/install/web_controller/share/web_controller/frontend/node_modules

.PHONY: run
run: # Runs the ROS targets
	# Run the web controller
	source setup_env.sh && ros2 launch hermes_robot all.launch.py 

.PHONY: run_validation
run_validation: # Runs the validation nodes
	# Ensure the setup script is sourced
	source setup_env.sh && ros2 launch hermes_validation_nodes validation.launch.py

