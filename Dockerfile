FROM ubuntu:22.04

RUN mkdir -p /basilisk

COPY ./ /basilisk/

WORKDIR /basilisk

RUN apt-get update -y && \
    apt-get install -y python3 \
        python3-pip \
        cmake \
        swig

RUN pip3 install pandas \
    numpy \
    matplotlib \
    pytest \
    Pillow \
    conan==1.59.0 \
    parse>=1.18.0 \
    cmake

RUN python3 conanfile.py

CMD ["tail", "-f", "/dev/null"]