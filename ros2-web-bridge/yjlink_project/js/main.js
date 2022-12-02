// Topic 선언부
var LaserScan = new ROSLIB.Topic({
    ros: ros,
    name: "/scan",
    messageType: "sensor_msgs/LaserScan"
});

var connect = false;
var turnon = new ROSLIB.Topic({
    ros: ros,
    name: "/turnon",
    messageType: "std_msgs/Bool"
});
var robot_state = document.getElementById("robot_state");
var robot_name = document.getElementById("robot_name");
turnon.subscribe((msg)=> {
    connect = msg.data;
});

setInterval(() => {
    connect? robot_state.innerText = "연결됨": robot_state.innerText = "연결 안됨";
    connect? robot_name.style.color = "green": robot_name.style.color = "red";
    ros2dmap();
}, 10);