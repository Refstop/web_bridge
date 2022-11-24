// Topic 선언부
var LaserScan = new ROSLIB.Topic({
    ros: ros,
    name: head_namespace + "/scan",
    messageType: "sensor_msgs/LaserScan"
});

var head_connect = false;
var head_turnon = new ROSLIB.Topic({
    ros: ros,
    name: head_namespace + "/turnon",
    messageType: "std_msgs/Bool"
});

var head_state = document.getElementById("head_state");
var head_name = document.getElementById("head_name");
head_turnon.subscribe((msg)=> {
    head_connect = msg.data;
    // console.log("head: " + String(head_connect));
});

var tail_connect = false;
var tail_turnon = new ROSLIB.Topic({
    ros: ros,
    name: "/turnon",
    messageType: "std_msgs/Bool"
});
var tail_state = document.getElementById("tail_state");
var tail_name = document.getElementById("tail_name");
tail_turnon.subscribe((msg)=> {
    tail_connect = msg.data;
    // console.log("tail: " + String(tail_connect));
});

createFunc(poseTopic, robotMarker, "nav");
setInterval(() => {
    head_connect? head_state.innerText = "연결됨": head_state.innerText = "연결 안됨";
    head_connect? head_name.style.color = "green": head_name.style.color = "red";
    tail_connect? tail_state.innerText = "연결됨": tail_state.innerText = "연결 안됨";
    tail_connect? tail_name.style.color = "green": tail_name.style.color = "red";
    ros2dmap();
}, 10);