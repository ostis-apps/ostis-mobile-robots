<h1 align="center">ostis-mobile-robots</h1>

[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Overview

`ostis-mobile-robots` is a working example of an ostis-system, utilizing the [OSTIS Technology](https://github.com/ostis-ai).

## Getting Started

Choose between Docker (recommended) or Native installation.

### Prerequisites

Ensure these tools are installed before proceeding:

#### General Prerequisites

Required for both Docker and Native installations:

*   **Git:**  For cloning the repository.
    [https://git-scm.com/book/en/v2/Getting-Started-Installing-Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

## Native Installation

Steps for installing and running the application directly on your system.

1.  **Install basic tools for development environment:**

    *   **Ubuntu/Debian (GCC):** 
        
        ```sh
        sudo apt update
        
        sudo apt install --yes --no-install-recommends \
            curl \
            ccache \
            python3 \
            python3-pip \
            build-essential \
            ninja-build
        ```
        
    *   **macOS (Clang):**

        ```sh
        brew update && brew upgrade
        brew install \
            curl \
            ccache \
            cmake \
            ninja
        ```

    *   **Other Linux distributions:**

        If you're using a different Linux distribution that doesn't support apt, ensure you have equivalent packages installed:

        * curl: A tool for transferring data with URLs;
        * ccache: A compiler cache to speed up compilation processes;
        * python3 and python3-pip: Python 3 interpreter and package installer;
        * build-essential: Includes a C++ compiler, necessary for building C++ components;
        * ninja-build: An alternative build system designed to be faster than traditional ones.

    Compiler is required for building C++ components.

2.  **Install pipx:**

    Instructions: [https://pipx.pypa.io/stable/installation/](https://pipx.pypa.io/stable/installation/).
    
    `pipx` isolates Python packages, preventing conflicts, especially useful when working with tools like CMake and Conan.

3.  **Install CMake:**

    ```sh
    pipx install cmake
    pipx ensurepath
    ```
   
    CMake is used to generate build files for your specific system. `pipx ensurepath` adds CMake to your PATH.

4.  **Install Conan:**

    ```sh
    pipx install conan
    pipx ensurepath
    ```
    
    Conan manages the project's C++ dependencies. `pipx ensurepath` adds Conan to your PATH.

5.  **Clone repository:**

    ```sh
    git clone https://github.com/ostis-apps/ostis-mobile-robots.git
    cd ostis-mobile-robots
    git submodule update --init --recursive
    ```

6.  **Restart your shell:**

    ```sh
    exec $SHELL
    ```
    
    Ensures that the PATH changes from `pipx ensurepath` are applied.

7.  **Install C++ problem solver dependencies:**

    They include sc-machine libraries -- the core components of the OSTIS Platform, used to develop C++ agents. They're installed using Conan:

    ```sh
    conan remote add ostis-ai https://conan.ostis.net/artifactory/api/conan/ostis-ai-library
    conan profile detect
    conan install . --build=missing
    ```
    
    `--build=missing` builds dependencies from source if pre-built binaries are not available.

8.  **Install sc-machine and scl-machine binaries:**
   
    sc-machine binaries are pre-compiled executables that provide the runtime environment for the ostis-system: build knowledge base source and launch the ostis-system. The installation process differs slightly between Linux and macOS:

    scl-machine binaries are pre-compiled modules with agents of logical inferences.
    
    ```sh
    ./scripts/install_cxx_problem_solver.sh
    ```
    
    Downloads and extracts pre-built `sc-machine` binaries for your operating system. The `include` directory is removed because it is not required.

9.  **Install sc-web:**

    sc-web provides the web-based user interface for the ostis-system. The installation process includes setting up dependencies and building the interface:

    *   **Ubuntu/Debian:**

        ```sh
        cd interface/sc-web
        ./scripts/install_deps_ubuntu.sh
        npm install  # Ensure npm dependencies are installed
        npm run build
        cd ../..
        ```

    *   **macOS:**

        ```sh
        cd interface/sc-web
        ./scripts/install_deps_macOS.sh
        npm install  # Ensure npm dependencies are installed
        npm run build
        cd ../..
        ```
    
    Installs the necessary dependencies for the web interface. `npm install` downloads JavaScript packages, and `npm run build` compiles the web interface.

10.  **Install dependencies for python agents:**

    ```sh
    cd problem-solver/py
    python3 -m venv .venv
    source .venv/bin/activate && pip3 install -r requirements.txt
    cd ../..
    ```

## Building ostis-system

1.  **Build problem solver:**
   
    The problem solver contains custom agents for your ostis-system. Build it using CMake:

    ```sh
    cmake --preset release-conan
    cmake --build --preset release
    ```
    
    These commands use CMake to build the C++ problem solver in Release mode. The `--preset` option specifies a pre-configured build setup.

2.  **Build knowledge base:**

    The knowledge base contains your custom knowledge represented in SC-code. It needs to be built before launching the system or after making changes:

    ```sh
    ./scripts/start.sh build_kb
    ```
    
    This command builds the knowledge base from the `.scs` and `.gwf` files in the `knowledge-base` directory, creating the `kb.bin` directory.

## Running ostis-system

1.  **Start `sc-machine` (in a terminal):**

    ```sh
    ./scripts/start.sh machine
    ```
    
    Starts the `sc-machine`, loading the knowledge base (`kb.bin`) and specifying the paths to the extensions.

2.  **Start `py-server` (in a separate terminal):**

    ```sh
    ./scripts/start.sh py_server
    ```
    
    Starts the server for python agents.

3.  **Start `sc-web` interface (in a separate terminal):**

    ```sh
    ./scripts/start.sh web
    ```
    
    Starts the web server.

4.  **Access interface:** Open `localhost:8000` in your web browser.

    ![Example Screenshot](https://i.imgur.com/6SehI5s.png)

To stop the running servers for the ostis-mobile-robots, press `Ctrl+C` in the terminals where sc-machine and sc-web are running.

## Documentation

To generate local documentation:

```sh
pip3 install mkdocs mkdocs-material markdown-include
mkdocs serve
```

Then open `http://127.0.0.1:8005/` in your browser.

*Note: The documentation is currently under development and may be incomplete.*

## Project Structure

*   **`knowledge-base`**: Contains the knowledge base source files (`.scs`, `.gwf`). Rebuild the knowledge base after making changes:

    ```sh
    ./scripts/start.sh build_kb
    ```

*   **`problem-solver`**: Contains the C++ agents that implement the problem-solving logic. Rebuild after modifying:

    ```sh
    cmake --preset release-conan
    cmake --build --preset release
    ```

    For debug mode:

    ```sh
    conan install . --build=missing -s build_type=Debug
    cmake --preset debug-conan
    cmake --build --preset debug
    ```

    For release mode with tests:

    ```sh
    cmake --preset release-with-tests-conan
    cmake --build --preset release
    ```

    To update log type and log level of agent, modify constructor of agent class. By default, logs of agents execution are stored in the `logs` directory:

    ```plain
    logs/
    |--ExampleAgent.log
    ```

## Code Style

This project follows the code style guidelines of `sc-machine`, which can be found [here](https://ostis-ai.github.io/sc-machine/dev/codestyle/).

## Author

*  GitHub: [@ostis-apps](https://github.com/ostis-apps), [@ostis-ai](https://github.com/ostis-ai)

## Support

Give us a ⭐️ if you like this project!

## Contributing

Contributions, issues, and feature requests are welcome! Check the [issues page](https://github.com/ostis-apps/ostis-mobile-robots/issues).

## License

This project is licensed under the [MIT License](https://opensource.org/license/mit/)
