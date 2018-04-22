FROM ubuntu:16.04

RUN apt-get update && \
    apt-get install --assume-yes --no-install-recommends \
        cmake \
        build-essential \
        mesa-utils \
        freeglut3-dev && \
    rm -rf /var/lib/apt/lists/*

# https://github.com/NVIDIA/nvidia-docker/issues/136#issuecomment-232755805
# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}    
