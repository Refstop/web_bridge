// robot의 pose를 arrow로 visualize하는 파트
var robotMarker = new ROS2D.NavigationArrow({
    size : 0.3,
    strokeSize : 0.01,
    fillColor: createjs.Graphics.getRGB(255, 0, 0, 0.9),
});

const createPoseTopic = (operatingMode) => {
    if(operatingMode == "slam") {
        const slamPoseTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/tf",
            messageType: "tf2_msgs/TFMessage"
        });
        return slamPoseTopic;
    } else if(operatingMode == "nav") {
        const navPoseTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/amcl_pose",
            messageType: "geometry_msgs/PoseWithCovarianceStamped"
        });
        return navPoseTopic;
    }
}
let poseTopic = createPoseTopic("nav");
const createFunc = function(poseTopic, robotMarker, operatingMode) {
    return poseTopic.subscribe(function(pose) {
        if(operatingMode == "slam") {
            let baselinkPose = pose.transforms[1].transform.translation

            let qz = pose.transforms[1].transform.rotation.z

            robotMarker.x = baselinkPose.x;
            robotMarker.y = -baselinkPose.y;

            let degz = 0;
            if(qz >= 0) {
                degz = qz / 1*180;
            } else {
                degz = (-qz) / 1*180 + 180;
            }
            robotMarker.rotation = degz;
        } else if(operatingMode == "nav") {
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
        }
        gridClient.rootObject.addChild(robotMarker);
    });
}