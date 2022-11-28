let mapTopic = new ROSLIB.Topic({
    ros: ros,
    topic: "omo/map",
    messageType: "std_msgs/Bool"
});
let width = 0;
let height = 0;
mapTopic.subscribe((msg) => {
    width = msg.info.width;
    height = msg.info.width;
});

let viewer = new ROS2D.Viewer({
    divID: "map",
    width: 508*2.5,
    height: 296*2.5
});

// map client 셋업
let gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    // topic: head_namespace + "/map",
    rootObject: viewer.scene,
    continuous: true
});

// map server로부터 publish된 map을 받아 viewer에 표시
function ros2dmap() {
    gridClient.on("change", function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
}