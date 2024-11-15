SHELL := /bin/bash

IMAGE_NAME := ros-humble-teleop

PREPARE_ROS := source /opt/ros/humble/setup.bash

# teleop config
TASK := bimanual_two_realsense_left_10fps

# record config
SAVE_BASE_DIR := /home/wendi/Desktop/record_data
SAVE_FILE_DIR := test
SAVE_FILE_NAME := trial1.pkl

docker.build:
	docker build -t ${IMAGE_NAME}:latest -f docker/Dockerfile .

docker.run:
	@if [ -z "$$(docker ps -q -f name=teleop)" ]; then \
		docker run -d \
			--runtime=nvidia \
			--gpus all \
			--network host \
			--privileged \
			-v .:/root/teleop \
			-v ${SAVE_BASE_DIR}:/root/record_data \
			-w /root/teleop \
			--name teleop \
			${IMAGE_NAME}:latest \
			tail -f /dev/null; \
	fi && \
	docker exec -it teleop bash

docker.remove:
	docker rm teleop

docker.clean:
	docker image rm ${IMAGE_NAME}:latest

docker.all: docker.build docker.run

teleop.launch_camera:
	. ../teleop-env/venv/bin/activate && \
	${PREPARE_ROS} && \
	python camera_node_launcher.py \
	task=${TASK}

teleop.launch_robot:
	. ../teleop-env/venv/bin/activate && \
	${PREPARE_ROS} && \
	python teleop.py \
	task=${TASK}

teleop.start_record:
	. ../teleop-env/venv/bin/activate && \
	${PREPARE_ROS} && \
	python record_data.py \
	    --save_base_dir /root/record_data \
	    --save_file_dir ${SAVE_FILE_DIR} \
	    --save_file_name ${SAVE_FILE_NAME} \
	    --save_to_disk

teleop.post_process:
	. ../teleop-env/venv/bin/activate && \
	${PREPARE_ROS} && \
	python post_process_data.py \
	--tag ${SAVE_FILE_DIR}