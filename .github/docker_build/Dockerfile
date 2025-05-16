FROM ubuntu:latest

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install necessary tools, including git and dependencies for the prerequisite script
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ccache \
    gawk \
    astyle \
    git \
    wget \
    python3 \
    python3-pip \
    make \
    libtool \
    flex \
    bison \
    autoconf \
    -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


RUN cd opt && \
    wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 && \
    tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 && \
    rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 && \
    rm -f gcc-arm-none-eabi-10-2020-q4-major/lib/libcc1.so && \
    rm -f gcc-arm-none-eabi-10-2020-q4-major/lib/libcc1.so.0 && \
    rm -f gcc-arm-none-eabi-10-2020-q4-major/lib/libcc1.so.0.0.0 && \
    cp -r gcc-arm-none-eabi-10-2020-q4-major/* /usr/

RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

RUN pip install --no-cache-dir \
    future \
    lxml \
    pymavlink \
    pyserial \
    MAVProxy \
    geocoder \
    empy==3.3.4 \
    ptyprocess \
    dronecan \
    pygame \
    intelhex


# Set the working directory
WORKDIR /ardupilot

# Command to keep the container running (optional, can be overridden by GitHub Actions)
CMD ["/bin/bash"]