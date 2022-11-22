var wp_check = $("input[id='wpmode']");
var wp_checkbox = document.getElementById("wpmode");
wp_check.click(function() {
    wp_checkbox.checked? document.getElementById("wp_cb_status").innerText = "ON": document.getElementById("wp_cb_status").innerText = "OFF";
    console.log("waypoint")
});

var wp_array = [];

// PoseArray를 받아 followWaypoints 함수에 넣어주는 노드를 작성
const createWaypoints = () => {
    if(!wp_checkbox.checked) return;
    const wparray = new ROSLIB.Topic({
        ros: ros,
        name: "/waypints_web",
        messageType: "geometry_msgs/PoseArray"
    });

    var wparray_msg = new ROSLIB.Message({
        header: {
            stamp: {
                sec: 0,
                nanosec: 100
            },
            frame_id: "map"
        },
        poses: wp_array
    });
    wparray.publish(wparray_msg);
    wp_checkbox.checked = !wp_checkbox.checked;
    wp_checkbox.checked? document.getElementById("wp_cb_status").innerText = "ON": document.getElementById("wp_cb_status").innerText = "OFF"; // wp_cb_status 추가
    wp_array = [];
};

document.getElementById("completebtn").onclick = function () {
    createWaypoints();
};