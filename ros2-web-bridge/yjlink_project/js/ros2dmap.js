let mapTopic = new ROSLIB.Topic({
    ros: ros,
    topic: "map",
    messageType: "nav_msgs/OccupancyGrid"
});
let map_width = 0;
let map_height = 0;
mapTopic.subscribe((msg) => {
    map_width = msg.info.width;
    map_height = msg.info.width;
});

let viewer = new ROS2D.Viewer({
    divID: "map",
    // width: 283*2.5,
    // height: 347*2.5
    width: 1024,
    height: 720
    
});

// map client 셋업
let gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
});

// Variables to store the starting position of the mouse
let startX = null;
let startY = null;

// Add an event listener for the "mousedown" event on the canvas element
document.getElementById('map').addEventListener('mousedown', function(event) {
    // Store the current position of the mouse as the starting position
    if(init_checkbox.checked || goal_checkbox.checked || wp_checkbox.checked) return;
    startX = event.clientX;
    startY = event.clientY;
});

// Add an event listener for the "mousemove" event on the canvas element
document.getElementById('map').addEventListener('mousemove', function(event) {
    if(init_checkbox.checked || goal_checkbox.checked || wp_checkbox.checked) return;
    if (startX !== null && startY !== null) {
        // Calculate the difference between the starting position and the current position
        let deltaX = -(event.clientX - startX)*0.0002;
        let deltaY = (event.clientY - startY)*0.0002;
        
        // Shift the map by the calculated difference
        viewer.shift(deltaX, deltaY);
    }
});

// Add an event listener for the "mouseup" event on the canvas element
document.getElementById('map').addEventListener('mouseup', function(event) {
    if(init_checkbox.checked || goal_checkbox.checked || wp_checkbox.checked) return;
    // Reset the starting position to null
    startX = null;
    startY = null;
});

// map server로부터 publish된 map을 받아 viewer에 표시
function ros2dmap() {
    gridClient.on("change", function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
}