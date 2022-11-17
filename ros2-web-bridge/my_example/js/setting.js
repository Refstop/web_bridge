// ROS web socket server configuration

var ros = new ROSLIB.Ros(); // rosbridge의 주소?? 정확한 설명 모름

ros.on("error", function(error) {
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "none";
    document.getElementById("error").style.display = "inline";
    console.log(error);
})

ros.on("connection", function() {
    console.log("Connection made!");
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "inline";
    document.getElementById("closed").style.display = "none";
    document.getElementById("error").style.display = "none";
});

ros.on("close", function() {
    console.log("Connection closed.");
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "inline";
})

// rosbridge websocket 서버와 연결을 생성
ros.connect("ws://localhost:9090");
