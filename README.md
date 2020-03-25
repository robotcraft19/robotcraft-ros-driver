[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<br />
<p align="center">
  <a href="https://github.com/robotcraft19/amazebot-ros-driver>
    <img src="res/images/logo_amazebot.png" alt="Logo" width="100" height="100">
    <img src="https://raw.githubusercontent.com/robotcraft19/amazebot-pathfinding-stage/master/res/images/logo_amazebot.png">
  </a>

  <h3 align="center">Amazebot Driver Package</h3>

  <p align="center">
    ROS Driver of the Amazebot. Data is shared between the Arduino Mega and the Raspberry Pi through the serial port. Several nodes are available, including the square test to highlight odometry errors, robot bringup, as well as the wallfollowing package applied on the actual robot.
    <br />
    <a href="https://github.com/robotcraft19/amazebot-ros-driver"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/robotcraft19/amazebot-ros-driver">View Demo</a>
    ·
    <a href="https://github.com/robotcraft19/amazebot-ros-driver/issues">Report Bug</a>
    ·
    <a href="https://github.com/robotcraft19/amazebot-ros-driver/issues">Request Feature</a>
  </p>
</p>

## Table of Contents

* [About the Project](#about-the-project)
* [Setup](#setup)
* [Run](#run)
  * [Reactive](#reactive-navigation)
  * [Teleoperation](#teleoperation)
* [Roadmap](#roadmap)
* [Contribute](#contribute)
* [License](#license)
* [Contact](#contact)
* [Contributors](#contributors)

## About the Project

<p align="center">
  <a href="https://github.com/robotcraft19/amazebot-ros-driver>
    <img src="res/images/maze.png" alt="Banner" width="400" height="260">
    <img src="https://raw.githubusercontent.com/robotcraft19/amazebot-ros-navigation/master/res/images/maze.png">
  </a>
</p>

This simulation package was developed during the Robotcraft '19 program. A basic right hand follower algorithm was provided by the supervisers but we decided to tune a P(ID) controller, which resulted in a smoother drive than anyone for the amazebot ! 

## Setup

To setup ROS, run the "install_melodic.sh" script in the scripts directory using `sh install_melodic.sh` or `chmod+x scripts/install_melodic.sh && ./ install_melodic.sh`

You can also use tutorials. There's a bunch of them, including in the ros wiki.

Now that you have ROS, to setup the project on your local machine:

1. Click on `Fork`.
2. Go to your fork and `clone` the project to your local machine, in the "catkin_ws" folder.
3. `git clone https://github.com/robotcraft19/amazebot-ros-driver.git`
4. `git clone https://github.com/robotcraft19/amazebot-model-description.git`
5. Make sure you have rosdep install : `sudo apt-get install python-rosdep && sudo rosdep init`
6. `cd ~/catkin_ws`
7. `rosdep install --from-paths src --ignore-src -r -y`
8. In the catkin workspace : `catkin_make`

If everything went smoothly, you should now have this repo's package as well as its dependencies.

The amazebot model description package is necessary to run simulations. The urdf description and rviz models are stored in this repository. (Yay you can see our robot !)

## Run

Running the nodes is quite easy as launch files were made. 

### Configuration :

- Amazebot Controller : `roslaunch amazebot-ros-driver amazebot.launch`
- Amazebot Bringup : `roslaunch amazebot-ros-driver robot_bringup.launch`
- Amazebot ROS Driver : `roslaunch amazebot-ros-driver robot_driver.launch`
- Amazebot Arduino Com : `roslaunch amazebot-ros-driver robot_firmware.launch`

### Tests :

- Stage Simulation : `roslaunch amazebot-ros-driver stage_test.launch`
- Draw A Square :  `roslaunch amazebot-ros-driver robot_test.launch`

### Maze Solver :

- Wallfollower :  `roslaunch amazebot-ros-driver wallfollower.launch`

## Roadmap

See the [open issues](https://github.com/robotcraft19/amazebot-ros-driver/issues) for a list of proposed features (and known issues).

## Contribute

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

### Contribute on proposed features

1. Choose any open issue from [here](https://github.com/robotcraft19/amazebot-ros-driver/issues). 
2. Comment on the issue: `Can I work on this?` and get assigned.
3. Make changes to your fork and send a PR.

Otherwise just create the issue yourself, and we'll discuss and assign you to it if serves the project !

To create a PR:

Follow the given link to make a successful and valid PR: https://help.github.com/articles/creating-a-pull-request/

To send a PR, follow these rules carefully, **otherwise your PR will be closed**:

1. Make PR title in this formats: 
```
Fixes #IssueNo : Name of Issue
``` 
```
Feature #IssueNo : Name of Issue
```
```
Enhancement #IssueNo : Name of Issue
```

According to what type of issue you believe it is.

For any doubts related to the issues, i.e., to understand the issue better etc, comment down your queries on the respective issue.

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Contact

Erwin Lejeune - [@spida_rwin](https://twitter.com/spida_rwin) - erwin.lejeune15@gmail.com

## Contributors

Everyone part of the original team or that assisted throughout the development.

- [Nicolas Filliol](https://github.com/nicofilliol)
- [Jan Tiepelt](https://github.com/jantiepelt)
- [Giovanni Alexander Bergamaschi](https://github.com/alexbergamaschi)
- [Oleksandr Koreiba](https://github.com/paradauz)
- [Erwin Lejeune](https://github.com/Guilyx)

[contributors-shield]: https://img.shields.io/github/contributors/robotcraft19/amazebot-ros-driver.svg?style=flat-square
[contributors-url]: https://github.com/robotcraft19/amazebot-ros-driver/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/robotcraft19/amazebot-ros-driver.svg?style=flat-square
[forks-url]: https://github.com/robotcraft19/amazebot-ros-driver/network/members
[stars-shield]: https://img.shields.io/github/stars/robotcraft19/amazebot-ros-driver.svg?style=flat-square
[stars-url]: https://github.com/robotcraft19/amazebot-ros-driver/stargazers
[issues-shield]: https://img.shields.io/github/issues/robotcraft19/amazebot-ros-driver.svg?style=flat-square
[issues-url]: https://github.com/robotcraft19/amazebot-ros-driver/issues
[license-shield]: https://img.shields.io/github/license/robotcraft19/amazebot-ros-driver.svg?style=flat-square
[license-url]: https://github.com/robotcraft19/amazebot-ros-driver/blob/master/LICENSE.md
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/erwinlejeune-lkn
