FROM ubuntu:18.04
MAINTAINER Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>

RUN apt-get update && \
    apt-get install \
    python3-pip \
    libspatialindex-dev \
    libgeos-dev \
    pybind11-dev -y

RUN pip3 install flake8 autopep8
    
COPY . /tmp/pcg_gazebo

WORKDIR /tmp/pcg_gazebo

RUN autopep8 --recursive --aggressive --diff --exit-code /tmp/pcg_gazebo/pcg_gazebo
RUN autopep8 --recursive --aggressive --diff --exit-code /tmp/pcg_gazebo/scripts
RUN autopep8 --recursive --aggressive --diff --exit-code /tmp/pcg_gazebo/tests

RUN flake8 /tmp/pcg_gazebo/pcg_gazebo
RUN flake8 /tmp/pcg_gazebo/scripts
RUN flake8 /tmp/pcg_gazebo/tests

RUN pip3 install -e .[all]

RUN python3 -c "import pcg_gazebo"

RUN pip3 install -e .[test]

RUN pytest -x /tmp/pcg_gazebo/tests