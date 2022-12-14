# ros2-web-bridge [![Build Status](https://travis-ci.org/RobotWebTools/ros2-web-bridge.svg?branch=develop)](https://travis-ci.org/RobotWebTools/ros2-web-bridge)[![Build status](https://ci.appveyor.com/api/projects/status/upb8xbq0f05mtgff/branch/develop?svg=true)](https://ci.appveyor.com/project/minggangw/ros2-web-bridge/branch/develop)[![npm](https://img.shields.io/npm/dt/ros2-web-bridge.svg)](https://www.npmjs.com/package/ros2-web-bridge)[![license](https://img.shields.io/github/license/RobotWebTools/ros2-web-bridge.svg)](https://github.com/RobotWebTools/ros2-web-bridge/blob/develop/LICENSE)

---

## :warning: Warning :warning:

This package is not actively maintained. Please use the [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) package instead for your ROS 2 websocket communication needs.

## ros2-web-bridge vs rosbridge_suite

- [`rosbridge_suite`](https://github.com/RobotWebTools/rosbridge_suite) (or `rosbridge_server`) is recommended for communicating with ROS 2 over websockets. It is written in Python and is actively maintained by the ROS web working group.
- [`ros2-web-bridge`](https://github.com/RobotWebTools/ros2-web-bridge) (this project) is an earlier attempt at enabling ROS 2 communication over websockets. It is written in JavaScript, and requires Node.js to be installed on your robot. It cannot be installed via `rosdep` or `apt` like a regular ROS package, and must be cloned and built locally.

---

## Server Implementations of the rosbridge v2 Protocol

ros2-web-bridge, which leverages the [rclnodejs](https://github.com/RobotWebTools/rclnodejs) client, provides a JSON interface to [ROS 2](https://index.ros.org/doc/ros2/) by adopting the [rosbridge v2 protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md). The bridge can process commands through JSON tuneled over WebSockets.

## ROS 2 support

The ros2-web-bridge **SUPPORTS** the latest ROS 2 stable release by default (currently [Dashing Patch 2](https://github.com/ros2/ros2/releases/tag/release-dashing-20190806)), please visit the [relase channel](https://github.com/ros2/ros2/releases) to check out the information.

Any one who wants to run on the nightly build of ROS 2, please change the `dependencies` section of [package.json](https://github.com/RobotWebTools/ros2-web-bridge/blob/develop/package.json) file to install other version of [rclnodejs](https://github.com/RobotWebTools/rclnodejs#match-with-ros-20-stable-releases).

## Supported Clients

A client is a program that communicates with ros2-web-bridge using its JSON API. Clients include:

* [roslibjs](https://github.com/RobotWebTools/roslibjs) - A JavaScript API, which communicates with ros2-web-bridge over WebSockets.

## Install

1. Prepare for ROS 2
    Please reference the [documentation](https://index.ros.org/doc/ros2/Installation/) to install ROS 2.
2. Install `Node.js`
    You can install Node.js:
    * Download from Node.js offical [website](https://nodejs.org/en/), and install it.
    * Use the Node Version Manager ([nvm](https://github.com/creationix/nvm)) to install it.
3. Clone and install dependencies
    Note that a ROS 2 installation has to be sourced before installing dependencies.
    ```bash
    $ git clone https://github.com/RobotWebTools/ros2-web-bridge.git
    $ cd ros2-web-bridge
    $ source /opt/ros/$DISTRO/setup.sh  # or a source installation
    $ npm install
    ```

## Run Examples

1. Make sure to source a ROS 2 installation, e.g.:
    ```bash
    $ source /opt/ros/$DISTRO/setup.sh  # or a source installation
    ```
2. Start `ros2-web-bridge` module:
    ```bash
    $ node bin/rosbridge.js
    ```
    If you want to start in client mode (i.e. connecting the bridge to an existing websocket server), do this instead:
    ```bash
    $ node bin/rosbridge.js --address ws://<address>:<port>
    ```
3. Start the [express](https://www.npmjs.com/package/express) server:
    ```bash
    $ cd examples && node index.js
    ```
4. Open your browser, and navigate to URL: http://localhost:3000/html/publisher.html

## Not supported `op`

Some experimental operations defined by rosbridge v2.0 protocol specification are not supported by ros2-web-bridge now, please check out the list:

* [fragment](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md#311-fragmentation--fragment--experimental)
* [png](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md#312-png-compression--png--experimental)
* [status](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md#322-status-message--status--experimental)

and the authentication

* [auth](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md#331-authenticate--auth-)

## Compability with rosbridge v2.0 protocol

We are trying to obey the [rosbridge v2 protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md), but there are still some operation commands which can not follow the spec. The table below lists the differences:

opreations | rosbridge v2.0 protocol spec | ros2-web-bridge implementation |
:------------: |  :------------ |  :------------- |
publish | If the msg is a subset of the type of the topic, then a warning status message is sent and the unspecified fields are filled in with [defaults](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md#343-publish--publish-). | If the subset of the msg is unspecified, then an error status message is sent and this message is dropped.
subscribe | The type of the topic is [optional](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md#344-subscribe). | The type of the topic must be offered.

If you use [roslibjs](https://static.robotwebtools.org/roslibjs/current/roslib.js) as the client running in the browser, please reference the code snippet below:

* Subscribe to a topic.
  ```JavaScript
  // Define a topic with its type.
  var example = new ROSLIB.Topic({
    ros : ros,
    name : '/example_topic',
    messageType : 'std_msgs/String'
  });

  // Subscribe to a topic.
  example.subscribe(function(message) {
    console.log(`Receive message: ${message}`);
  });
  ```

## Contributing

If you want to contribute code to this project, first you need to fork the
project. The next step is to send a pull request (PR) for review. The PR will be reviewed by the project team members. Once you have gained "Look Good To Me (LGTM)", the project maintainers will merge the PR.

## License

This project abides by [Apache License 2.0](https://github.com/RobotWebTools/ros2-web-bridge/blob/develop/LICENSE).
