ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
IMAGE_NAME:=swarmslam
CONTAINER_NAME:=rob530
DOCKERFILE_NAME:=./Dockerfile

image:
	docker build $(ROOT_DIR) -t $(IMAGE_NAME) -f $(DOCKERFILE_NAME)

container:
	docker create \
		--name $(CONTAINER_NAME) \
		--volume $(ROOT_DIR):/ros_ws/src/swarmslam \
		--interactive \
		--publish 6080:80 \
		$(IMAGE_NAME):latest

start:
	docker start $(CONTAINER_NAME)

stop:
	docker stop $(CONTAINER_NAME)

rm: stop
	docker rm $(CONTAINER_NAME)

shell: start
	docker exec -it $(CONTAINER_NAME) bash
