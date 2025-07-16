# RAMSAI

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/ICube-Robotics/RAMSAI/actions/workflows/ci.yml/badge.svg)](https://github.com/ICube-Robotics/RAMSAI/actions/workflows/ci.yml)

Github repository for the ANR RAMSAI project.

# Getting started


***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).

2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.

3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```

3. Clone this repository:
    ```shell
    git clone https://github.com/ICube-Robotics/RAMSAI.git
    ```

> [!NOTE]  
> Checkout to specific application (i.e., branch) if needed...

4. Pull relevant packages, install dependencies:
    ```shell
    cd RAMSAI
    mkdir external
    vcs import ./external < ramsai.repos
    rosdep install --ignore-src --from-paths . -y -r
    ```

6. Compile and source the workspace by using:
    ```shell
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```

## About

**For more information, please check the [documentation](https://icube-robotics.github.io/RAMSAI/).**

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Laurent Barbé:__ [laurent.barbe@unistra.fr](mailto:laurent.barbe@unistra.fr)
__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](https://github.com/mcbed)
