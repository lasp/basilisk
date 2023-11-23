FROM ubuntu:jammy

ENV DEBIAN_FRONTEND noninteractive

#RUN apt-get update && apt-get install --no-install-recommends -y build-essential \
#        cmake \
#        git \
#        git-lfs \
#        gtk2.0 \
#        libgtk2.0-dev \
#        ninja-build \
#        openssh-client \
#        python3-dev \
#        python-is-python3 \
#        python3-pip \
#        python3-setuptools \
#        python3-tk \
#        swig \
#        vim \
#        vim-youcompleteme \
#        && rm -rf /var/lib/apt/lists/*
#
#RUN apt-get update && apt-get install --no-install-recommends -y g++-9 \
#    gcc-9 \
#    && rm -rf /var/lib/apt/lists/*
#RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 --slave /usr/bin/g++ g++ /usr/bin/g++-9

#RUN adduser basiliskuser
#USER basiliskuser
#WORKDIR /home/basiliskuser
#
#RUN git clone https://github.com/lasp/basilisk.git
#
#WORKDIR /home/basiliskuser/basilisk
#
## The local path has to be added otherwise the pip install command in the conanfile
## won't install Basilisk to where?? fix this rationale write up
#ENV PATH="/home/basiliskuser/.local/bin:${PATH}"
#
#RUN python -m pip install --upgrade pip
#RUN python -m pip install conan==1.61.0 pandas parse matplotlib numpy Pillow pytest pytest-xdist
#
#ENV CONAN_REVISIONS_ENABLED=1
#ENV PYTHONUNBUFFERED=1
#RUN conan remote add bincrafters https://bincrafters.jfrog.io/artifactory/api/conan/public-conan
#RUN conan install . --build=missing -s build_type=Release -if dist3/conan \
#    -o opNav=True \
#    -o vizInterface=False \
#    -o generator="Ninja"
#
#RUN conan build . -if dist3/conan
