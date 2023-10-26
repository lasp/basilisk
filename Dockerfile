FROM ubuntu:22.04

RUN apt-get update -y && \
    apt-get install -y python3 \
        python3-pip \
        python3-venv \
        cmake \
        swig \
        git \
        sudo

RUN cd $HOME \
    && python3 -m venv venv && \
    . venv/bin/activate && \
    pip install pandas \
    numpy \
    matplotlib \
    pytest \
    Pillow \
    conan==1.59.0 \
    parse>=1.18.0 \
    cmake

# RUN cd $HOME \
#     && git clone https://github.com/lasp/basilisk.git

COPY ./ /root/basilisk

# RUN cd /root \
#     && . venv/bin/activate \
#     && cd /root/basilisk \
#     && python conanfile.py

CMD ["tail", "-f", "/dev/null"]