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

var head_status = document.getElementById("head_status");
var head_name = document.getElementById("head_name");
head_turnon.subscribe((msg)=> {
    head_connect = msg.data;
});

var tail_connect = false;
var tail_turnon = new ROSLIB.Topic({
    ros: ros,
    name: "/turnon",
    messageType: "std_msgs/Bool"
});
var tail_status = document.getElementById("tail_status");
var tail_name = document.getElementById("tail_name");
tail_turnon.subscribe((msg)=> {
    tail_connect = msg.data;
});

let tail_nav = "tracking";

setInterval(() => {
    head_connect? head_name.style.color = "green": head_name.style.color = "red";
    tail_connect? tail_name.style.color = "green": tail_name.style.color = "red";
    
    if(head_state == STATUS.WAITING) {
        head_status.innerText = "대기";
    } else if(head_state == STATUS.DEPART_BACK) {
        head_status.innerText = "복귀";
    } else if(head_state == HEAD_POINT) {
        head_status.innerText = "도착";
    }

    if(tail_state == STATUS.WAITING) {
        tail_status.innerText = "대기";
    } else if(tail_state == STATUS.DEPART_BACK) {
        tail_status.innerText = "복귀";
    } else if(tail_state == STATUS.ROOM_POINT) {
        tail_status.innerText = "도착";
    }

    if(tail_mode == STATUS.TRACKING) {
        tail_nav = "tracking";
    } else if(tail_mode == STATUS.NAVIGATING) {
        tail_nav = "navigating";
    }
    let nav_mode_msg = new ROSLIB.Message({
        data: tail_nav
    });
    nav_mode.publish(nav_mode_msg);
    ros2dmap();
}, 10);