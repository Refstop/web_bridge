// 토글 스위치를 사용하여, initial pose 모드와 goal pose 모드를 결정
let init_check = $("input[id='initmode']");
let init_checkbox = document.getElementById("initmode");
let init_cb_status = document.getElementById("init_cb_status");
init_check.click(function() {
    init_checkbox.checked? init_cb_status.innerText = "ON": init_cb_status.innerText = "OFF";
});

let goal_check = $("input[id='goalmode']");
let goal_checkbox = document.getElementById("goalmode");
let goal_cb_status = document.getElementById("goal_cb_status")
goal_check.click(function() {
    goal_checkbox.checked? goal_cb_status.innerText = "ON": goal_cb_status.innerText = "OFF";
});

// initial pose와 goal_pose를 publish하는 함수
let status_area = document.getElementById("status_area");
const createInitialPose = (pose_x, pose_y, thetaRad) => {
    if(!init_checkbox.checked) return;
    const initialPose = new ROSLIB.Topic({
        ros: ros,
        name: "/initialpose",
        messageType: "geometry_msgs/PoseWithCovarianceStamped"
    });

    let qz = Math.sin(-thetaRad/2.0);
    let qw = Math.cos(-thetaRad/2.0);
    let orientation = new ROSLIB.Quaternion({
        x: 0, y: 0, z: qz, w: qw
    });

    let initialpose_msg = new ROSLIB.Message({
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
    let str = "초기 Pose: " + String(pose_x.toFixed(3)) + " " + String(pose_y.toFixed(3)) + " " + String(thetaRad.toFixed(3)) + "\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
    if(init_checkbox.checked) {
        init_checkbox.checked = !init_checkbox.checked;
        init_checkbox.checked? init_cb_status.innerText = "ON": init_cb_status.innerText = "OFF";
    }
};

const createGoalPose = (pose_x, pose_y, thetaRad) => {
    if(!goal_checkbox.checked) return;
    
    const goal_pose = new ROSLIB.Topic({
        ros: ros,
        name: "/goal_pose",
        messageType: "geometry_msgs/PoseStamped"
    });

    let qz = Math.sin(-thetaRad/2.0);
    let qw = Math.cos(-thetaRad/2.0);
    let orientation = new ROSLIB.Quaternion({
        x: 0, y: 0, z: qz, w: qw
    });

    let goal_pose_msg = new ROSLIB.Message({
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
    let str = "목표 Pose: " + String(pose_x.toFixed(3)) + " " + String(pose_y.toFixed(3)) + " " + String(thetaRad.toFixed(3)) + "\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
    if(goal_checkbox.checked) {
        goal_checkbox.checked = !goal_checkbox.checked;
        goal_checkbox.checked? goal_cb_status.innerText = "ON": goal_cb_status.innerText = "OFF";
    }
    
};

// mouse 이벤트를 통해 initial pose와 goal pose를 publish하는 함수를 실행
let mouseDown = false;
let mouseDownPose = {};

let targetMarker = new ROS2D.NavigationArrow({
    size : 0.3,
    strokeSize : 0.01,
    fillColor: createjs.Graphics.getRGB(0, 255, 0, 0.9),
});

let mouseEventHandler = function(event, mouseState, operMode) {
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
            waypointHandler.addWaypoint(mouseDownPose.position.x, mouseDownPose.position.y, thetaRad);
            let str = String(waypointHandler.wp_array.length) + "번째 waypoint 저장됨\n";
            status_area.value += str;
            status_area.scrollTop = status_area.scrollHeight;
        }
        gridClient.rootObject.removeChild(targetMarker);
    }
}

// waypoint handler 추가
viewer.scene.addEventListener("stagemousedown", function(event) {
    if(init_checkbox.checked) {    
        mouseEventHandler(event, "down", "initial");
    } else if(goal_checkbox.checked) {        
        mouseEventHandler(event, "down", "goal");
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "down", "wpsave");
    }
});

viewer.scene.addEventListener("stagemousemove", function(event) {
    if(init_checkbox.checked) {    
        mouseEventHandler(event, "move", "initial");
    } else if(goal_checkbox.checked) {        
        mouseEventHandler(event, "move", "goal");
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "move", "wpsave");
    }
});

viewer.scene.addEventListener("stagemouseup", function(event) {
    if(init_checkbox.checked) {
        mouseEventHandler(event, "up", "initial");
    } else if(goal_checkbox.checked) {        
        mouseEventHandler(event, "up", "goal");
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "up", "wpsave");
    }
});

document.getElementById('map').addEventListener('wheel', function(event) {
    if(event.deltaY < 0) {
        viewer.scene.scaleX *= 1.25;
        viewer.scene.scaleY *= 1.25;
    } else {
        viewer.scene.scaleX *= 0.8;
        viewer.scene.scaleY *= 0.8;
    }
    
});