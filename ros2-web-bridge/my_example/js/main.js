// ROS web socket server configuration

var ros = new ROSLIB.Ros({
    url: 'ws://0.0.0.0:9090'
}); // rosbridge의 주소?? 정확한 설명 모름

ros.on("connection", function() {
    console.log("Connection made!");
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "none";
    document.getElementById("error").style.display = "inline";
});

ros.on("close", function() {
    console.log("Connection closed.");
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "inline";
    document.getElementById("error").style.display = "none";
})

ros.on("error", function() {
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "none";
    document.getElementById("error").style.display = "inline";
    console.log(error);
})

// rosbridge websocket 서버와 연결을 생성
ros.connect("ws://localhost:9090");

var viewer = new ROS2D.Viewer({
    divID: "map",
    width: 700,
    height: 700
});

// Setup the map client.
var gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
});

function mapload() {
    gridClient.on('change', function(){
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
}

window.addEventListener('onload', console.log("map load!"), mapload())