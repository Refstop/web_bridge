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

var viewer = new ROS2D.Viewer({
    divID: "map",
    width: 508*2,
    height: 296*2
});

// Setup the map client.
var gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
})

var robotMarker = new ROS2D.NavigationArrow({
    size : 0.5,
    strokeSize : 0.01,
    pulse: true,
    fillColor: createjs.Graphics.getRGB(255,0,0, 0.9),
});
gridClient.rootObject.addChild(robotMarker);

robotMarker.x = 0;
robotMarker.y = 0;
robotMarker.scaleX = 0.01;
robotMarker.scaleY = 0.01;
robotMarker.rotation = -90;
robotMarker.visible = true;

let oldPose = {x: 0.0, y: 0.0};

update = function(pose, rotation) {
    robotMarker.x = pose.x;
    robotMarker.y = -pose.y;
    
    // robotMarker.rotation = viewer2D.scene.rosQuaternionToGlobalTheta(orientation);
    robotMarker.rotation = rotation;
    if (oldPose.x !== pose.x || oldPose.y !== pose.y) {
        viewer2D.shift(-oldPose.x + pose.x, -oldPose.y + pose.y);
        oldPose = pose;
    }
};

const createInitialPose = (pose_x, pose_y, orientation)=>{
    if(!init_checkbox.checked) return;
    const initialPose = new ROSLIB.Topic({
        ros: ros,
        name: "/initialpose",
        messageType: "geometry_msgs/PoseWithCovarianceStamped"
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
    document.getElementById("targetpose").innerText = " initialpose => px: " + String(pose_x) + " py: " + String(pose_y);
    init_checkbox.checked = !init_checkbox.checked;
    init_checkbox.checked? document.getElementById("init_cb_status").innerText = "ON": document.getElementById("init_cb_status").innerText = "OFF";
};

const createGoalPose = (pose_x, pose_y, orientation) => {
    if(!goal_checkbox.checked) return;
    const goal_pose = new ROSLIB.Topic({
        ros: ros,
        name: "/goal_pose",
        messageType: "geometry_msgs/PoseStamped"
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

var check = $("input[type='checkbox']");
var init_checkbox = document.getElementById("init");
check.click(function() {
    init_checkbox.checked? document.getElementById("init_cb_status").innerText = "ON": document.getElementById("init_cb_status").innerText = "OFF";
});

var goal_checkbox = document.getElementById("goal");
check.click(function() {
    goal_checkbox.checked? document.getElementById("goal_cb_status").innerText = "ON": document.getElementById("goal_cb_status").innerText = "OFF";
});

let mouseDown = false;
let mouseDownPose = {};

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

        var qz = Math.sin(-thetaRad/2.0);
        var qw = Math.cos(-thetaRad/2.0);
        var orientation = new ROSLIB.Quaternion({
            x: 0, y: 0, z: qz, w: qw
        });

        if(operMode == "initial") {
            createInitialPose(mouseDownPose.position.x, mouseDownPose.position.y, orientation);
        } else if(operMode == "goal") {
            createGoalPose(mouseDownPose.position.x, mouseDownPose.position.y, orientation);
        }
    }
}

viewer.scene.addEventListener("stagemousedown", function(event) {
    if(init_checkbox.checked) {
        mouseEventHandler(event, "down", "initial");
    } else {
        if(goal_checkbox.checked) {
            mouseEventHandler(event, "down", "goal");
        }
    }
})

viewer.scene.addEventListener("stagemousemove", function(event) {
    if(init_checkbox.checked) {
        mouseEventHandler(event, "move", "initial");
    } else {
        if(goal_checkbox.checked) {
            mouseEventHandler(event, "move", "goal");
        }
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
});

function ros2dmap() {
    gridClient.on("change", function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
    update({x:0, y:0}, 1.57)
}

window.addEventListener("onload", console.log("map load!"), ros2dmap())