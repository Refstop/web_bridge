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

function ros2dmap() {
    gridClient.on('change', function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
}

window.addEventListener('onload', console.log("map load!"), mapload())