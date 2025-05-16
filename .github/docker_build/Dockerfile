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


RUN python -m venv /opt/venv
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