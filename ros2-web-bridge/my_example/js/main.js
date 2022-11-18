// Topic 선언부
var GlobalPath = new ROSLIB.Topic({
    ros: ros,
    name: "/plan",
    messageType: "nav_msgs/Path"
});

var LocalPath = new ROSLIB.Topic({
    ros: ros,
    name: "/local_plan",
    messageType: "nav_msgs/Path"
});

var LaserScan = new ROSLIB.Topic({
    ros: ros,
    name: "/scan",
    messageType: "sensor_msgs/LaserScan"
});

createFunc(poseTopic, robotMarker, "nav");

setInterval(() => {
    ros2dmap();
}, 10);