FROM ubuntu:22.04 as base

RUN apt-get update -y && \
    apt-get install -y python3 \
        python3-pip \
        python3-venv \
        cmake \
        swig \
        git \
        sudo

RUN cd $HOME \
    && git clone -b feature/build-fsw-only https://github.com/dinkelk/basilisk.git
    #&& cd basilisk \
    ##&& git config pull.rebase false \
    #&& git fetch \
    #&& git checkout -b feature/build-fsw-only origin/feature/build-fsw-only
    #&& git pull origin feature/build-fsw-only

RUN cd $HOME/basilisk && \
    python3 -m venv venv && \
    . venv/bin/activate && \
    python3 -m pip install --upgrade pip && \
    pip install pandas \
      numpy \
      matplotlib \
      pytest \
      Pillow \
      conan==1.59.0 \
      parse>=1.18.0 \
      cmake

RUN cd $HOME/basilisk \
    && git branch -v \
    && rm -rf ~/.cache/pip/selfcheck/ \
    && . venv/bin/activate \
    #&& conan install . --build=missing -s build_type=Debug -if dist3/conan -o opNav=True -o vizInterface=True -o generator="Ninja" \
    #&& conan build . -if dist3/conan
    && python conanfile.py --opNav True --vizInterface True

CMD ["tail", "-f", "/dev/null"]

# get and build GCC for the RISC-V
FROM base as install_riscv-gcc-imfc
RUN apt-get update \
  && apt-get install -yq autoconf \
    automake \
    autotools-dev \
    curl \
    python3 \
    python3-pip \
    libmpc-dev \
    libmpfr-dev \
    libgmp-dev \
    gawk \
    build-essential \
    bison \
    flex \
    texinfo \
    gperf \
    libtool \
    patchutils \
    bc \
    zlib1g-dev \
    libexpat-dev \
    ninja-build \
    git \
    cmake \
    libglib2.0-dev
ENV RISCV="/opt/riscv_imfc"
ENV PATH="$RISCV/bin$PATH"
WORKDIR $RISCV
RUN git clone --recursive https://github.com/riscv/riscv-gnu-toolchain \
    && cd riscv-gnu-toolchain  \
    && ./configure --prefix=$RISCV --with-arch=rv32imfc --with-abi=ilp32f --enable-multilib \
    && make

# get and build GCC for the RISC-V
FROM base as install_riscv-gcc-imaf
RUN apt-get update \
  && apt-get install -yq autoconf \
    automake \
    autotools-dev \
    curl \
    python3 \
    python3-pip \
    libmpc-dev \
    libmpfr-dev \
    libgmp-dev \
    gawk \
    build-essential \
    bison \
    flex \
    texinfo \
    gperf \
    libtool \
    patchutils \
    bc \
    zlib1g-dev \
    libexpat-dev \
    ninja-build \
    git \
    cmake \
    libglib2.0-dev
ENV RISCV="/opt/riscv_imaf"
ENV PATH="$RISCV/bin$PATH"
WORKDIR $RISCV
RUN git clone --recursive https://github.com/riscv/riscv-gnu-toolchain \
    && cd riscv-gnu-toolchain  \
    && ./configure --prefix=$RISCV --with-arch=rv32imaf --with-abi=ilp32f --enable-multilib \
    && make

FROM base AS final
COPY --from=install_riscv-gcc-imfc /opt/riscv_imfc /opt/riscv_imfc
COPY --from=install_riscv-gcc-imaf /opt/riscv_imaf /opt/riscv_imaf
