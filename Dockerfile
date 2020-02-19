ARG cuda_version=9.0
ARG cudnn_version=7
FROM nvidia/cuda:${cuda_version}-cudnn${cudnn_version}-devel

# Install system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
      bzip2 \
      g++ \
      git \
      graphviz \
      libgl1-mesa-glx \
      libhdf5-dev \
      openmpi-bin \
      sudo \
      graphviz \
      python-pip \
      curl \
      python-tk \
      xvfb \
      mesa-utils \
      libegl1-mesa \
      libgl1-mesa-glx \
      libglu1-mesa \
      libx11-6 \
      libav-tools \
      imagemagick \
      x11-common \
      x11-xserver-utils \
      wget && \
    rm -rf /var/lib/apt/lists/*

# Now get Drake and stuff
ARG DRAKE_URL

#RUN curl -o drake.tar.gz $DRAKE_URL && sudo tar -xzf drake.tar.gz -C /opt
COPY drake_install/ /opt/drake
RUN yes | sudo /opt/drake/share/drake/setup/install_prereqs
RUN git clone https://github.com/RussTedrake/underactuated /underactuated
RUN yes | sudo /underactuated/scripts/setup/ubuntu/16.04/install_prereqs

# Install conda
ENV CONDA_DIR /opt/conda
ENV PATH $CONDA_DIR/bin:$PATH

RUN wget --quiet --no-check-certificate https://repo.continuum.io/miniconda/Miniconda3-4.2.12-Linux-x86_64.sh && \
    echo "c59b3dd3cad550ac7596e0d599b91e75d88826db132e4146030ef471bb434e9a *Miniconda3-4.2.12-Linux-x86_64.sh" | sha256sum -c - && \
    /bin/bash /Miniconda3-4.2.12-Linux-x86_64.sh -f -b -p $CONDA_DIR && \
    rm Miniconda3-4.2.12-Linux-x86_64.sh && \
    echo export PATH=$CONDA_DIR/bin:'$PATH' > /etc/profile.d/conda.sh

# Install Python packages and keras
ENV NB_USER keras
ENV NB_UID 1000

RUN useradd -m -s /bin/bash -N -u $NB_UID $NB_USER && \
    chown $NB_USER $CONDA_DIR -R && \
    mkdir -p /src && \
    chown $NB_USER /src

USER $NB_USER

ARG python_version=2.7

RUN conda install -y python=${python_version} && \
    pip install --upgrade pip && \
    pip install \
      sklearn_pandas \
      tensorflow-gpu \
      meshcat \
      imageio \
      noise && \
    conda install \
      bcolz \
      h5py \
      matplotlib \
      mkl \
      nose \
      notebook \
      Pillow \
      pandas \
      pygpu \
      pyyaml \
      scikit-learn \
      jupyter \
      graphviz \
      numpy \
      scipy \
      opencv \
      matplotlib \
      six && \
    git clone git://github.com/keras-team/keras.git /src && pip install -e /src[tests] && \
    pip install git+git://github.com/keras-team/keras.git && \
    conda clean -yt

ENV PYTHONPATH='/src/workspace/DepthSim/python:/underactuated/src:/opt/drake/lib/python2.7/site-packages:/src/:$PYTHONPATH'

WORKDIR /src

EXPOSE 8888 6000 7000
