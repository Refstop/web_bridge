var wp_check = $("input[id='wpmode']");
var wp_checkbox = document.getElementById("wpmode");
wp_check.click(function() {
    wp_checkbox.checked? document.getElementById("wp_cb_status").innerText = "ON": document.getElementById("wp_cb_status").innerText = "OFF";
});

var wp_array = [];
// PoseArray를 받아 followWaypoints 함수에 넣어주는 노드를 작성
const createWaypoints = () => {
    if(wp_array.length == 0) return;
    const wparray = new ROSLIB.Topic({
        ros: ros,
        name: "/waypoints_web",
        messageType: "geometry_msgs/PoseArray"
    });

    var wparray_msg = new ROSLIB.Message({
        header: {
            stamp: {
                sec: 0,
                nanosec: 100
            },
            frame_id: "map"
        },
        poses: wp_array
    });
    wparray.publish(wparray_msg);
    if(wp_checkbox.checked) {
        wp_checkbox.checked = !wp_checkbox.checked;
        wp_checkbox.checked? document.getElementById("wp_cb_status").innerText = "ON": document.getElementById("wp_cb_status").innerText = "OFF"; // wp_cb_status 추가
    }
    wp_array = [];
    var str = "Waypoint Navigation 시작\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
};

document.getElementById("completebtn").onclick = function () {
    createWaypoints();
};

document.getElementById("clearbtn").onclick = function () {
    wp_array = [];
    var str = "Waypoints 삭제 완료\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
};


let markerInfo = new Map();
let marker = document.getElementById("marker_id");

// Parse room information from the file data
function parseMarkerInfo(fileData) {
    let arr = fileData.split('"');
    let xyth = arr[2].split(' ');

    // Store room information in the Map
    markerInfo.set(
        arr[0], [parseFloat(xyth[1]), parseFloat(xyth[2]), parseFloat(xyth[6]), parseFloat(xyth[7])]
    );
    for(let val of markerInfo.get(arr[0])) {
        console.log(val)
    }
    

    // Create an option element for the room
    let option = document.createElement('option');
    option.value = arr[0];
    option.innerText = arr[1];
    marker.append(option);
}

document.getElementById('input_file').addEventListener('change', function() {
    let file = new FileReader();

    file.onload = () => {
        let fileData = file.result;
        fileData = fileData.split('\n');
        
        // Parse information for each room
        for(let i = 0; i < fileData.length; i++) {
            parseMarkerInfo(fileData[i]);
        }
    };
    file.readAsText(this.files[0]);
});

function createWaypointsByFile() {
    let dest = markerInfo.get(document.getElementById("marker_id").value);
    let orientation = new ROSLIB.Quaternion({
        x: 0.0, y: 0.0, z: dest[2], w: dest[3]
    });
    wp_array.push({
        position: {
            x: dest[0],
            y: dest[1],
            z: 0
        },
        orientation: orientation
    });
    createWaypoints();
}
document.getElementById("startbtn").onclick = createWaypointsByFile;