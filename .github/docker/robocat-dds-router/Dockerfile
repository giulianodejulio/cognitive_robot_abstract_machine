# %% build dependencies
FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates cmake g++ git make libasio-dev libboost-dev \
    libtinyxml2-dev libssl-dev libyaml-cpp-dev libsqlite3-dev \
    && rm -rf /var/lib/apt/lists/*

# %% pinned upstream sources
WORKDIR /build
COPY sources.tsv /build/sources.tsv
RUN while read -r directory repository revision; do \
      git clone --depth 1 --branch "$revision" "$repository" "/build/src/$directory"; \
    done < /build/sources.tsv

# %% sequential compilation
ENV CMAKE_PREFIX_PATH=/opt/ddsrouter
RUN set -eu; \
    for package in \
      foonathan_memory_vendor fastcdr fastdds \
      dev-utils/cmake_utils dev-utils/cpp_utils \
      ddspipe/ddspipe_core ddspipe/ddspipe_participants ddspipe/ddspipe_yaml \
      ddsrouter/ddsrouter_core ddsrouter/ddsrouter_yaml ddsrouter/tools/ddsrouter_tool; do \
      cmake -S "/build/src/$package" -B /build/current \
        -DCMAKE_INSTALL_PREFIX=/opt/ddsrouter \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG' \
        -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DBUILD_SHARED_LIBS=ON \
        -DCOMPILE_EXAMPLES=OFF -DCOMPILE_TOOLS=OFF -DINSTALL_TOOLS=OFF \
        -DFASTDDS_STATISTICS=OFF -DSQLITE3_SUPPORT=OFF; \
      cmake --build /build/current --parallel 1; \
      cmake --install /build/current --strip; \
      rm -rf /build/current; \
    done

# %% runtime image
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libtinyxml2-10 libssl3t64 libyaml-cpp0.8 libstdc++6 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/ddsrouter/bin/ddsrouter /usr/local/bin/ddsrouter
COPY --from=build /opt/ddsrouter/lib/ /opt/ddsrouter/lib/
RUN printf '/opt/ddsrouter/lib\n' > /etc/ld.so.conf.d/ddsrouter.conf && ldconfig
USER 65534:65534
ENTRYPOINT ["/usr/local/bin/ddsrouter"]
CMD ["--help"]
