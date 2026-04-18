FROM ubuntu:noble AS base

ENV CCACHE_DIR=/ccache
USER root

COPY scripts /ostis-mobile-robots/scripts
COPY conanfile.py /ostis-mobile-robots/conanfile.py
COPY CMakePresets.json /ostis-mobile-robots/CMakePresets.json
COPY CMakeLists.txt /ostis-mobile-robots/CMakeLists.txt
COPY requirements.txt /ostis-mobile-robots/requirements.txt

# tini is an init system to forward interrupt signals properly
RUN apt update && apt install -y --no-install-recommends sudo tini curl ccache python3 python3-pip pipx cmake build-essential ninja-build

# Install Conan
RUN pipx install conan && \
    pipx ensurepath

FROM base AS devdeps
WORKDIR /ostis-mobile-robots

SHELL ["/bin/bash", "-c"]
RUN python3 -m venv /ostis-mobile-robots/.venv && \
    source /ostis-mobile-robots/.venv/bin/activate && \
    pip3 install -r /ostis-mobile-robots/requirements.txt

ENV PATH="/root/.local/bin:$PATH"
RUN conan remote add ostis-ai https://conan.ostis.net/artifactory/api/conan/ostis-ai-library && \
    conan profile detect && \
    conan install . --build=missing

# Install sc-machine binaries
RUN ./scripts/install_cxx_problem_solver.sh

FROM devdeps AS devcontainer
RUN apt install -y --no-install-recommends cppcheck valgrind gdb bash-completion ninja-build curl
ENTRYPOINT ["/bin/bash"]

FROM devdeps AS builder
COPY . .
RUN --mount=type=cache,target=/ccache/ cmake --preset release-conan && cmake --build --preset release

# Gathering all artifacts together
FROM base AS final

COPY --from=builder /ostis-mobile-robots/scripts /ostis-mobile-robots/scripts
COPY --from=builder /ostis-mobile-robots/install /ostis-mobile-robots/install
COPY --from=builder /ostis-mobile-robots/build/Release/extensions /ostis-mobile-robots/build/Release/extensions
COPY --from=builder /ostis-mobile-robots/.venv /ostis-mobile-robots/.venv

WORKDIR /ostis-mobile-robots

EXPOSE 8090

ENTRYPOINT ["/usr/bin/tini", "--", "/ostis-mobile-robots/scripts/docker_entrypoint.sh"]
