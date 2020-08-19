FROM ubuntu:18.04
MAINTAINER Leonardo Edgar

ARG USER
ARG GROUP
ARG UID
ARG GID

# Install core linux tools
RUN apt-get update && apt-get install -y --no-install-recommends \
	apt-utils lsb-release sudo unzip wget ssh vim curl ca-certificates gnupg \
	&& rm -rf /var/lib/apt/lists/*

# Install python packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
	python3-numpy python3-matplotlib python3-tk --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*

# User and permissions
ARG HOME=/home/${USER}
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${GID} ${GROUP} \
    && useradd -d ${HOME} -u ${UID} -g ${GID} -m -s /bin/bash ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sudoers_${USER}
USER ${USER}


WORKDIR ${HOME}
