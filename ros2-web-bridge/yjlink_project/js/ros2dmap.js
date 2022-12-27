// var mapTopic = new ROSLIB.Topic({
//     ros: ros,
//     topic: "/map",
//     messageType: "std_msgs/Bool"
// });
// var width = 0;
// var height = 0;
// mapTopic.subscribe((msg) => {
//     width = msg.info.width;
//     height = msg.info.width;
// });

var viewer = new ROS2D.Viewer({
    divID: "map",
    width: 508*2.5,
    height: 296*2.5
});

// map client 셋업
var gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
});

// Set up the mouse wheel event handler
viewer.scene.addEventListener('wheel', function(event) {
    console.log(event.deltaY);
    // Check the delta value to determine the scroll direction
    if (event.deltaY > 0) {
        // Zoom out by decreasing the scale value
        viewer.scene.scaleX *= 0.5;
        viewer.scene.scaleY *= 0.5;
    } else {
        // Zoom in by increasing the scale value
        viewer.scene.scaleX *= 2;
        viewer.scene.scaleY *= 2;
    }
});

// map server로부터 publish된 map을 받아 viewer에 표시
function ros2dmap() {
    gridClient.on("change", function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
}