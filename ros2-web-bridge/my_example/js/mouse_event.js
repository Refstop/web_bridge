// 토글 스위치를 사용하여, initial pose 모드와 goal pose 모드를 결정
// var check = $("input[type='checkbox']");
var init_check = $("input[id='initmode']");
var init_checkbox = document.getElementById("initmode");
init_check.click(function() {
    init_checkbox.checked? document.getElementById("init_cb_status").innerText = "ON": document.getElementById("init_cb_status").innerText = "OFF";
    console.log("init");
});

var goal_check = $("input[id='goalmode']");
var goal_checkbox = document.getElementById("goalmode");
goal_check.click(function() {
    goal_checkbox.checked? document.getElementById("goal_cb_status").innerText = "ON": document.getElementById("goal_cb_status").innerText = "OFF";
    console.log("goal");
});

// initial pose와 goal_pose를 publish하는 함수
const createInitialPose = (pose_x, pose_y, thetaRad)=>{
    if(!init_checkbox.checked) return;
    const initialPose = new ROSLIB.Topic({
        ros: ros,
        name: "/initialpose",
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
    // gridClient.rootObject.addChild(LaserScan);
    initialPose.publish(initialpose_msg);
    document.getElementById("targetpose").innerText = " initialpose => px: " + String(pose_x) + " py: " + String(pose_y);
    init_checkbox.checked = !init_checkbox.checked;
    init_checkbox.checked? document.getElementById("init_cb_status").innerText = "ON": document.getElementById("init_cb_status").innerText = "OFF";
};

const createGoalPose = (pose_x, pose_y, thetaRad) => {
    if(!goal_checkbox.checked) return;
    const goal_pose = new ROSLIB.Topic({
        ros: ros,
        name: "/goal_pose",
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
    document.getElementById("targetpose").innerText = " goal_pose => px: " + String(pose_x) + " py: " + String(pose_y);
    goal_checkbox.checked = !goal_checkbox.checked;
    goal_checkbox.checked? document.getElementById("goal_cb_status").innerText = "ON": document.getElementById("goal_cb_status").innerText = "OFF";
};

// mouse 이벤트를 통해 initial pose와 goal pose를 publish하는 함수를 실행
let mouseDown = false;
let mouseDownPose = {};

// var targetMarker = new ROS2D.NavigationArrow({
//     size : 0.3,
//     strokeSize : 0.01,
//     fillColor: createjs.Graphics.getRGB(0, 255, 0, 0.9),
// });

var mouseEventHandler = function(event, mouseState, operMode) {
    // console.log("mouseState: " + mouseState);
    if(mouseState == "down") {
        mouseDown = true;
        // console.log("mouse down");
        // get position when mouse button is pressed down
        mouseDownPosition = viewer.scene.globalToRos(event.stageX, event.stageY);
        mouseDownPositionVec3 = new ROSLIB.Vector3(mouseDownPosition);
        mouseDownPose = new ROSLIB.Pose({
            position: new ROSLIB.Vector3(mouseDownPositionVec3)
        });
        console.log(mouseDownPose.position);
    }
    else if(mouseState === "move" && mouseDown) {
        // console.log("mouse move");
        // remove obsolete orientation marker
        gridClient.rootObject.removeChild(robotMarker)
    }
    else if(mouseState === "up" && mouseDown) {
        mouseDown = false;
        mouseUpPosition = viewer.scene.globalToRos(event.stageX, event.stageY);
        mouseUpPositionVec3 = new ROSLIB.Vector3(mouseUpPosition);
        const mouseUpPose = new ROSLIB.Pose({
            position: new ROSLIB.Vector3(mouseUpPositionVec3)
        });

        // upPose - DownPose
        xDelta = mouseUpPose.position.x - mouseDownPose.position.x
        yDelta = mouseUpPose.position.y - mouseDownPose.position.y
        thetaRad = Math.atan2(xDelta, yDelta);
        thetaDeg = thetaRad * (180.0/Math.PI);
        if(thetaRad >= 0 && thetaRad <= Math.PI) {
            thetaRad += (3*Math.PI/2);
        } else {
            thetaRad -= Math.PI/2;
        }
        // gridClient.rootObject.addChild(targetMarker);

        if(operMode == "initial") {
            createInitialPose(mouseDownPose.position.x, mouseDownPose.position.y, thetaRad);
        } else if(operMode == "goal") {
            createGoalPose(mouseDownPose.position.x, mouseDownPose.position.y, thetaRad);
        } else if(operMode == "wpsave") {
            console.log("saved")
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
            })
            document.getElementById("completebtn").innerText = "Play!!\nLength: " + String(wp_array.length);
        }
    }
}

// waypoint handler 추가
viewer.scene.addEventListener("stagemousedown", function(event) {
    if(init_checkbox.checked) {
        mouseEventHandler(event, "down", "initial");
    } else {
        if(goal_checkbox.checked) {
            mouseEventHandler(event, "down", "goal");
        }
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "down", "wpsave");
    }
});

viewer.scene.addEventListener("stagemousemove", function(event) {
    if(init_checkbox.checked) {
        mouseEventHandler(event, "move", "initial");
    } else {
        if(goal_checkbox.checked) {
            mouseEventHandler(event, "move", "goal");
        }
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "move", "wpsave");
    }
});

viewer.scene.addEventListener("stagemouseup", function(event) {
    if(init_checkbox.checked) {
        mouseEventHandler(event, "up", "initial");
    } else {
        if(goal_checkbox.checked) {
            mouseEventHandler(event, "up", "goal");
        }
    }
    if(wp_checkbox.checked) {
        mouseEventHandler(event, "up", "wpsave");
    }
});