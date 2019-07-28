FROM kurorororo/downward-lab-experiment

RUN apt update -y && apt install -y \
    wget \
    unzip \
    libboost-all-dev \
    libjemalloc-dev \
    time \
    && apt clean

WORKDIR /pplanner
ADD CMakeLists.txt /pplanner/
ADD src /pplanner/src
ADD tests /pplanner/tests
ADD googletest /pplanner/googletest

WORKDIR /pplanner/lib
RUN cd /pplanner/lib
RUN wget http://www.tcs.hut.fi/Software/bliss/bliss-0.73.zip
RUN unzip bliss-0.73.zip
RUN mv bliss-0.73 bliss
RUN cd /pplanner/lib/bliss && make

WORKDIR /pplanner/build
RUN cmake ..
RUN make planner

WORKDIR /pplanner