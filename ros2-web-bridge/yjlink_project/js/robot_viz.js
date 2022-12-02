// robot의 pose를 arrow로 visualize하는 파트
const createPoseTopic = () => {
    const navPoseTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/amcl_pose",
        messageType: "geometry_msgs/PoseWithCovarianceStamped"
    });
    return navPoseTopic;
}

const createFunc = function(poseTopic, robotMarker) {
    return poseTopic.subscribe(function(pose) {
        if(pose.header.frame_id == "map") {
            robotMarker.x = pose.pose.pose.position.x;
            robotMarker.y = -pose.pose.pose.position.y;

            let orientationQuart = pose.pose.pose.orientation;
            var q0 = orientationQuart.w;
            var q1 = orientationQuart.x;
            var q2 = orientationQuart.y;
            var q3 = orientationQuart.z;
            deg = -Math.atan2(2 * (q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0 / Math.PI;
            robotMarker.rotation = deg;
        }
        gridClient.rootObject.addChild(robotMarker);
    });
}

// path 시각화
var listenerforPath = new ROSLIB.Topic ({
    ros : ros,
    name : '/local_plan',
    messageType : 'nav_msgs/Path'
});
gridClient.rootObject.addChild(pathShape);

var pathShape = new ROS2D.PathShape({
    strokeSize : 0.1,
    strokeColor : createjs.Graphics.getRGB(0, 255, 0,1),
});
    
gridClient.rootObject.addChild(pathShape);

listenerforPath.subscribe((message)=> {
    if(message) {
        pathShape.setPath(message);
    }
});

var robotMarker = new ROS2D.NavigationArrow({
    size : 0.3,
    strokeSize : 0.01,
    fillColor: createjs.Graphics.getRGB(255, 0, 0, 0.9),
});

let poseTopic = createPoseTopic();
createFunc(poseTopic, robotMarker);