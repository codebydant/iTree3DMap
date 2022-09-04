# Base stage
FROM ghcr.io/danieltobon43/pcl-docker:latest AS base
RUN apt-get update && apt-get install -y --no-install-recommends \
	libopencv-dev \	
	&& rm -rf /var/lib/apt/lists/*

FROM base AS build
# install compilation tools
RUN apt-get update && apt-get install -y --no-install-recommends \
	cmake \
	build-essential \
	git \
	gcc-multilib \
	g++-multilib \
	&& rm -rf /var/lib/apt/lists/*
# install openMVG dependency
RUN cd /tmp && git clone --recursive https://github.com/openMVG/openMVG && \
	cd /tmp/openMVG/src && \
	cmake -DCMAKE_INSTALL_PREFIX=/tmp/openMVG/install \
	-DOpenMVG_BUILD_TESTS=OFF \
	-DOpenMVG_BUILD_EXAMPLES=OFF \
	-DTARGET_ARCHITECTURE=generic \
	-S . -Bbuild && make -C build/ -j$(nproc) --no-print-directory && \
	make -C build/ install --no-print-directory
# install CMVS-PMV dependency
RUN cd /tmp && git clone --recursive https://github.com/pmoulon/CMVS-PMVS.git && \
	cd /tmp/CMVS-PMVS/program && \
	cmake -DCMAKE_INSTALL_PREFIX=/tmp/CMVS-PMVS/install \
	-S . -Bbuild && make -C build/ -j$(nproc) --no-print-directory && \
	make -C build/ install --no-print-directory
# install TinyXML-2 dependency
RUN cd /tmp && git clone --recursive https://github.com/leethomason/tinyxml2 && \
	cd /tmp/tinyxml2 && \
	cmake -DCMAKE_INSTALL_PREFIX=/tmp/tinyxml2/install \
	-S . -Bbuild && make -C build/ -j$(nproc) --no-print-directory && \
	make -C build/ install --no-print-directory

FROM base AS runtime
COPY --from=build /tmp/openMVG/install /usr
COPY --from=build /tmp/CMVS-PMVS/install /usr
COPY --from=build /tmp/tinyxml2/install /usr

# development
FROM runtime AS dev
# install compilation tools
RUN apt-get update && apt-get install -y --no-install-recommends \
	cmake \
	build-essential \
	git \
	&& rm -rf /var/lib/apt/lists/*
