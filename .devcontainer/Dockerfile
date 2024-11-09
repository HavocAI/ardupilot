ARG BASE_IMAGE="ubuntu"
ARG TAG="22.04"
FROM ${BASE_IMAGE}:${TAG}
WORKDIR /ardupilot

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ARG SKIP_AP_EXT_ENV=1
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1

RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    usbutils \
    gdb-multiarch \
    bash-completion

COPY Tools/environment_install/install-prereqs-ubuntu.sh /ardupilot/Tools/environment_install/
COPY Tools/completion /ardupilot/Tools/completion/

# Install JLink drivers
COPY Drivers/JLink_Linux_V810b_x86_64.deb /opt
RUN apt-get install -y                                                      \ 
    libxrender1 libxcb-render0 libxcb-render-util0 libxcb-shape0            \ 
    libxcb-randr0 libxcb-xfixes0 libxcb-sync1 libxcb-shm0 libxcb-icccm4     \
    libxcb-keysyms1 libxcb-image0 libxkbcommon0 libxkbcommon-x11-0          \
    libfontconfig1 libfreetype6 libxext6 libx11-xcb1 libsm6 libice6         \
    libglib2.0-0                                                            \
    && dpkg --unpack /opt/JLink_Linux_V810b_x86_64.deb                      \    
    && rm -f /var/lib/dpkg/info/jlink.postinst                              \
    && dpkg --configure jlink                                               \
    && apt-get install -yf

# Install debug helper scripts
COPY server.sh /usr/local/bin/server
COPY debug.sh /usr/local/bin/debug
RUN chmod +x /usr/local/bin/server && chmod +x /usr/local/bin/debug 

# Install shell banner
RUN tee --append /home/${USER_NAME}/.bashrc <<EOL
echo "## HavocAI ArduoPilot development container"
echo "##"
echo "## Configure:          './waf configure --board <board> --debug'"
echo "## Build:              './waf build rover'"
echo "## Start GDBServer:    'server'"
echo "## Start GDB:          'debug'"
echo ""
EOL

# Create non root user for pip
ENV USER=${USER_NAME}

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}

ENV SKIP_AP_EXT_ENV=$SKIP_AP_EXT_ENV SKIP_AP_GRAPHIC_ENV=$SKIP_AP_GRAPHIC_ENV SKIP_AP_COV_ENV=$SKIP_AP_COV_ENV SKIP_AP_GIT_CHECK=$SKIP_AP_GIT_CHECK
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# add waf alias to ardupilot waf to .ardupilot_env
RUN echo "alias waf=\"/${USER_NAME}/waf\"" >> ~/.ardupilot_env

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

# Create entrypoint as docker cannot do shell substitution correctly
RUN export ARDUPILOT_ENTRYPOINT="/home/${USER_NAME}/ardupilot_entrypoint.sh" \
    && echo "#!/bin/bash" > $ARDUPILOT_ENTRYPOINT \
    && echo "set -e" >> $ARDUPILOT_ENTRYPOINT \
    && echo "source /home/${USER_NAME}/.ardupilot_env" >> $ARDUPILOT_ENTRYPOINT \
    && echo 'exec "$@"' >> $ARDUPILOT_ENTRYPOINT \
    && chmod +x $ARDUPILOT_ENTRYPOINT \
    && sudo mv $ARDUPILOT_ENTRYPOINT /ardupilot_entrypoint.sh

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

# Cleanup
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV CCACHE_MAXSIZE=1G
ENTRYPOINT ["/ardupilot_entrypoint.sh"]
CMD ["bash"]
