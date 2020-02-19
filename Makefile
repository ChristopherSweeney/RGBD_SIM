help:
	@cat Makefile

GPU?=0
DOCKER_FILE=Dockerfile
DOCKER=GPU=$(GPU) nvidia-docker
BACKEND=tensorflow
PYTHON_VERSION?=2.7
CUDA_VERSION?=9.0
CUDNN_VERSION?=7
SRC?=$(shell dirname `pwd`)
DRAKE_URL="https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz"
IMAGE_NAME=notavailableinallareas/pydrake_rgbd_sim_keras:20180614

build:
	docker build -t notavailableinallareas/pydrake_rgbd_sim_keras --build-arg DRAKE_URL=$(DRAKE_URL) --build-arg python_version=$(PYTHON_VERSION) --build-arg cuda_version=$(CUDA_VERSION) --build-arg cudnn_version=$(CUDNN_VERSION) -f $(DOCKER_FILE) .

publish: build
	docker tag notavailableinallareas/pydrake_rgbd_sim_keras notavailableinallareas/pydrake_rgbd_sim_keras:`date +'%Y%m%d'`
	docker push notavailableinallareas/pydrake_rgbd_sim_keras:`date +'%Y%m%d'`

bash:
	xhost +local:root
	$(DOCKER) run -it -v $(SRC)/pydrake_rgbd_sim_anon:/src/workspace --env KERAS_BACKEND=$(BACKEND) --net="host" -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw $(IMAGE_NAME) bash
