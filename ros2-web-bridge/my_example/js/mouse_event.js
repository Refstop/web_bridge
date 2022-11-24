// 토글 스위치를 사용하여, initial pose 모드와 goal pose 모드를 결정
var head_init_check = $("input[id='head_initmode']");
var head_init_checkbox = document.getElementById("head_initmode");
var head_init_cb_status = document.getElementById("head_init_cb_status");
head_init_check.click(function() {
    head_init_checkbox.checked? head_init_cb_status.innerText = "ON": head_init_cb_status.innerText = "OFF";
    if(head_init_checkbox.checked) tail_init_checkbox.checked = false;
});

var head_goal_check = $("input[id='head_goalmode']");
var head_goal_checkbox = document.getElementById("head_goalmode");
var head_goal_cb_status = document.getElementById("head_goal_cb_status");
head_goal_check.click(function() {
    head_goal_checkbox.checked? head_goal_cb_status.innerText = "ON": head_goal_cb_status.innerText = "OFF";
    if(head_goal_checkbox.checked) tail_goal_checkbox.checked = false;
});

var tail_init_check = $("input[id='tail_initmode']");
var tail_init_checkbox = document.getElementById("tail_initmode");
var tail_init_cb_status = document.getElementById("tail_init_cb_status");
tail_init_check.click(function() {
    tail_init_checkbox.checked? tail_init_cb_status.innerText = "ON": tail_init_cb_status.innerText = "OFF";
    if(tail_init_checkbox.checked) head_init_checkbox.checked = false;
});

var tail_goal_check = $("input[id='tail_goalmode']");
var tail_goal_checkbox = document.getElementById("tail_goalmode");
var tail_goal_cb_status = document.getElementById("tail_goal_cb_status")
tail_goal_check.click(function() {
    tail_goal_checkbox.checked? tail_goal_cb_status.innerText = "ON": tail_goal_cb_status.innerText = "OFF";
    if(tail_goal_checkbox.checked) head_goal_checkbox.checked = false;
});

// initial pose와 goal_pose를 publish하는 함수
var status_area = document.getElementById("status_area");
const createInitialPose = (pose_x, pose_y, thetaRad) => {
    if(head_init_checkbox.checked) {
        var robot_type = head_namespace;
    } else if(tail_init_checkbox.checked) {
        var robot_type = tail_namespace;
    } else if(!head_init_checkbox.checked) {
        return;
    }
    const initialPose = new ROSLIB.Topic({
        ros: ros,
        name: robot_type + "/initialpose",
        messageType: "geometry_msgs/PoseWithCovarianceStamped"
    });

    var qz = Math.sin(-thetaRad/2.0);
    var qw = Math.cos(-thetaRad/2.0);
    var orientation = new ROSLIB.Quaternion({
        x: 0, y: 0, z: qz, w: qw
    });

    var initialpose_msg = new ROSLIB.Message({
        header: {
            stamp: {
                sec: 0,
                nanosec: 100
            },
            frame_id: "map"
        },
        pose: {
            pose: {
                position: {
                    x: pose_x,
                    y: pose_y,
                    z: 0
                },
                orientation: orientation
            },
            covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        }
    });
    initialPose.publish(initialpose_msg);
    var str = robot_type + " 초기 Pose: " + String(pose_x.toFixed(3)) + " " + String(pose_y.toFixed(3)) + " " + String(thetaRad.toFixed(3)) + "\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
    // head_init_checkbox.checked = !head_init_checkbox.checked;
    // head_init_checkbox.checked? head_init_cb_status.innerText = "ON": head_init_cb_status.innerText = "OFF";
    if(head_init_checkbox.checked) {
        head_init_checkbox.checked = !head_init_checkbox.checked;
        head_init_checkbox.checked? head_init_cb_status.innerText = "ON": head_init_cb_status.innerText = "OFF";
    } else if(tail_init_checkbox.checked) {
        tail_init_checkbox.checked = !tail_init_checkbox.checked;
        tail_init_checkbox.checked? tail_init_cb_status.innerText = "ON": tail_init_cb_status.innerText = "OFF";
    }
};

const createGoalPose = (pose_x, pose_y, thetaRad) => {
    if(head_goal_checkbox.checked) {
        var robot_type = head_namespace;
    } else if(tail_goal_checkbox.checked) {
        var robot_type = tail_namespace;
    } else if(!head_goal_checkbox.checked) {
        return;
    } 
    const goal_pose = new ROSLIB.Topic({
        ros: ros,
        name: robot_type + "/goal_pose",
        messageType: "geometry_msgs/PoseStamped"
    });

    var qz = Math.sin(-thetaRad/2.0);
    var qw = Math.cos(-thetaRad/2.0);
    var orientation = new ROSLIB.Quaternion({
        x: 0, y: 0, z: qz, w: qw
    });

    var goal_pose_msg = new ROSLIB.Message({
        header: {
            stamp: {
                sec: 0,
                nanosec: 100
            },
            frame_id: "map"
        },
        pose: {
            position: {
                x: pose_x,
                y: pose_y,
                z: 0
            },
            orientation: orientation
        }
    });
    goal_pose.publish(goal_pose_msg);
    var str = robot_type + " 목표 Pose: " + String(pose_x.toFixed(3)) + " " + String(pose_y.toFixed(3)) + " " + String(thetaRad.toFixed(3)) + "\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
    // head status: 배송 중/배송 대기/배송 완료
    // tail status: 배송 중/배송 완료
    // head 배송 시작(tail 추종 중)-head 배송 중-head 도착-tail 출발-head 배송 대기-tail 배송 중
    // -(사용자 확인)-tail 배송 완료-tail 복귀 중-tail 복귀 완료-head 배송 완료
    if(head_goal_checkbox.checked) {
        head_goal_checkbox.checked = !head_goal_checkbox.checked;
        head_goal_checkbox.checked? head_goal_cb_status.innerText = "ON": head_goal_cb_status.innerText = "OFF";
    } else if(tail_goal_checkbox.checked) {
        tail_goal_checkbox.checked = !tail_goal_checkbox.checked;
        tail_goal_checkbox.checked? tail_goal_cb_status.innerText = "ON": tail_goal_cb_status.innerText = "OFF";
    }
    
};

// mouse 이벤트를 통해 initial pose와 goal pose를 publish하는 함수를 실행
let mouseDown = false;
let mouseDownPose = {};

var targetMarker = new ROS2D.NavigationArrow({
    size : 0.3,
    strokeSize : 0.01,
    fillColor: createjs.Graphics.getRGB(0, 255, 0, 0.9),
});

var mouseEventHandler = function(event, mouseState, operMode) {
    // console.log("mouseState: " + mouseState);
    if(mouseState == "down") {
        mouseDown = true;
        // get position when mouse button is pressed down
        mouseDownPosition = viewer.scene.globalToRos(event.stageX, event.stageY);
        mouseDownPositionVec3 = new ROSLIB.Vector3(mouseDownPosition);
        mouseDownPose = new ROSLIB.Pose({
            position: new ROSLIB.Vector3(mouseDownPositionVec3)
        });
    }
    else if(mouseState === "move" && mouseDown) {
        // remove obsolete orientation marker
        mouseMovePosition = viewer.scene.globalToRos(event.stageX, event.stageY);
        mouseMovePositionVec3 = new ROSLIB.Vector3(mouseMovePosition);
        const mouseMovePose = new ROSLIB.Pose({
            position: new ROSLIB.Vector3(mouseMovePositionVec3)
        });
        xDelta = mouseMovePose.position.x - mouseDownPose.position.x
        yDelta = mouseMovePose.position.y - mouseDownPose.position.y
        thetaRad = Math.atan2(xDelta, yDelta);
        thetaDeg = thetaRad * (180.0/Math.PI);
        if(thetaRad >= 0 && thetaRad <= Math.PI) {
            thetaRad += (3*Math.PI/2);
        } else {
            thetaRad -= Math.PI/2;
        }

        targetMarker.x = mouseDownPose.position.x;
        targetMarker.y = -mouseDownPose.position.y;
        targetMarker.rotation = thetaRad * 180.0 / Math.PI;
        gridClient.rootObject.addChild(targetMarker);
        
    }
    else if(mouseState === "up" && mouseDown) {
        mouseDown = false;

        if(operMode == "initial") {
            createInitialPose(mouseDownPose.position.x, mouseDownPose.position.y, thetaRad);
        } else if(operMode == "goal") {
            createGoalPose(mouseDownPose.position.x, mouseDownPose.position.y, thetaRad);
        } else if(operMode == "wpsave") {
            var qz = Math.sin(-thetaRad/2.0);
            var qw = Math.cos(-thetaRad/2.0);
            var orientation = new ROSLIB.Quaternion({
                x: 0, y: 0, z: qz, w: qw
            });
            wp_array.push({
                position: {
                    x: mouseDownPose.position.x,
                    y:mouseDownPose.position.y,
                    z: 0
                },
                orientation: orientation
            });
            var str = String(wp_array.length) + "번째 waypoint 저장됨\n";
            status_area.value += str;
            status_area.scrollTop = status_area.scrollHeight;
        }
        gridClient.rootObject.removeChild(targetMarker);
    }
}

// waypoint handler 추가
viewer.scene.addEventListener("stagemousedown", function(event) {
    if(head_init_checkbox.checked) {
        mouseEventHandler(event, "down", "initial");
    } else if(head_goal_checkbox.checked) {        
        mouseEventHandler(event, "down", "goal");
    } else if(tail_init_checkbox.checked) {    
        mouseEventHandler(event, "down", "initial");
    } else if(tail_goal_checkbox.checked) {        
        mouseEventHandler(event, "down", "goal");
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "down", "wpsave");
    }
});

viewer.scene.addEventListener("stagemousemove", function(event) {
    if(head_init_checkbox.checked) {
        mouseEventHandler(event, "move", "initial");
    } else if(head_goal_checkbox.checked) {        
        mouseEventHandler(event, "move", "goal");
    } else if(tail_init_checkbox.checked) {    
        mouseEventHandler(event, "move", "initial");
    } else if(tail_goal_checkbox.checked) {        
        mouseEventHandler(event, "move", "goal");
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "move", "wpsave");
    }
});

viewer.scene.addEventListener("stagemouseup", function(event) {
    if(head_init_checkbox.checked) {
        mouseEventHandler(event, "up", "initial");
    } else if(head_goal_checkbox.checked) {        
        mouseEventHandler(event, "up", "goal");
    } else if(tail_init_checkbox.checked) {    
        mouseEventHandler(event, "up", "initial");
    } else if(tail_goal_checkbox.checked) {        
        mouseEventHandler(event, "up", "goal");
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "up", "wpsave");
    }
});