FROM condaforge/miniforge3

RUN conda install -y -c conda-forge gdal pdal compilers make cmake ninja

SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]

RUN wget https://github.com/gadomski/fgt/archive/v0.4.8.tar.gz && \
    tar xvf v0.4.8.tar.gz && \
    cd fgt-0.4.8 && \
    mkdir build && \
    cd build &&  \
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX .. && \
    ninja install && \
    cd / && rm -rf fgt*

RUN wget https://github.com/gadomski/cpd/archive/v0.5.3.tar.gz && \
    tar zxvf v0.5.3.tar.gz && \
    cd cpd-0.5.3 && \
    mkdir build && \
    cd build && \
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX .. && \
    ninja install && \
    cd / && rm -rf cpd*

ADD . /

RUN make

ENTRYPOINT ["/atlas-cpd"]

