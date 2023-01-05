function yaw2quat(yaw) {
    let qz = Math.sin(-yaw/2.0);
    let qw = Math.cos(-yaw/2.0);
    return [0.0, 0.0, qz, qw];
}

function quat2yaw(quat) {
    let siny_cosp = 2 * (quat[3] * quat[2]);
    let cosy_cosp = 1 - 2 * (quat[2] * quat[2]);
    return -Math.atan2(siny_cosp, cosy_cosp);
}

function quat_mul(quat0, quat1) {
    x0 = quat0[0]; x1 = quat1[0];
    y0 = quat0[1]; y1 = quat1[1];
    z0 = quat0[2]; z1 = quat1[2];
    w0 = quat0[3]; w1 = quat1[3];
    
    return [x1*w0 + y1*z0 - z1*y0 + w1*x0,
            -x1*z0 + y1*w0 + z1*x0 + w1*y0,
            x1*y0 - y1*x0 + z1*w0 + w1*z0,
            -x1*x0 - y1*y0 - z1*z0 + w1*w0];
}

let marker_list = document.getElementById("marker_list");
let initTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/initialize",
    messageType: "std_msgs/String"
});
initTopic.subscribe((msg) => {
    for(let info of msg.data.split(" ")) {
        let option = document.createElement('option');
        option.value = info.split("_")[0];
        option.innerText = info.split("_")[1];
        marker_list.append(option);
    }
    let str = "Waypoint Load 완료\n";
    status_area.value += str;
    status_area.scrollTop = status_area.scrollHeight;
});


function requestWaypoints(e) {
    let displayRequest = new ROSLIB.Topic({
        ros: ros,
        name: "/display_req",
        messageType: "std_msgs/Int8"
    });
    let displayDestinationMessage = new ROSLIB.Message({
        data: parseInt(e.value)
    });
    displayRequest.publish(displayDestinationMessage);
}
let displayResponse = new ROSLIB.Topic({
    ros: ros,
    name: "/display_resp",
    messageType: "geometry_msgs/PoseArray"
});
displayResponse.subscribe((msg) => {
    waypointHandler.viewWaypoints(msg.poses);
});

class WaypointHandler {
    constructor() {
        this.wp_array = [];
        this.arrow_arr = [];
    }

    addWaypoint(x, y, thetaRad) {
        let quat = yaw2quat(thetaRad);
        let orientation = new ROSLIB.Quaternion({
            x: 0, y: 0, z: quat[2], w: quat[3]
        });
        this.wp_array.push({
            position: {
                x: x,
                y: y,
                z: 0
            },
            orientation: orientation
        });
        let wp_arrow = new ROS2D.NavigationArrow({
            size : 0.2,
            strokeSize : 0.01,
            fillColor: createjs.Graphics.getRGB(0, 0, 255, 0.9)
        });
        wp_arrow.x = x;
        wp_arrow.y = -y;
        wp_arrow.rotation = thetaRad * 180.0/Math.PI;

        gridClient.rootObject.addChild(wp_arrow);
        this.arrow_arr.push(wp_arrow);

        let elements = document.getElementsByClassName("marker_info");
        for(let i = 0; i < elements.length; i++) {
            elements.item(i).disabled = true;
        }
    }

    removeWaypoint() {

    }

    createWaypoints() {
        if(this.wp_array.length == 0) return;
        const wparray = new ROSLIB.Topic({
            ros: ros,
            name: "/waypoints_web",
            messageType: "geometry_msgs/PoseArray"
        });
        
        let wparray_msg = new ROSLIB.Message({
            header: {
                stamp: {
                    sec: 0,
                    nanosec: 100
                },
                frame_id: "map"
            },
            poses: this.wp_array
        });
        wparray.publish(wparray_msg);
        if(wp_checkbox.checked) {
            wp_checkbox.checked = !wp_checkbox.checked;
            wp_checkbox.checked? document.getElementById("wp_cb_status").innerText = "ON": document.getElementById("wp_cb_status").innerText = "OFF"; // wp_cb_status 추가
        }
        let str = "Waypoint Navigation 시작\n";
        status_area.value += str;
        status_area.scrollTop = status_area.scrollHeight;
        this.clearWaypoints();
    }

    createWaypointsByFile() {
        let dest = markerInfo.get(marker_list.value);
        let thetaRad = quat2yaw([dest[3], dest[4], dest[5], dest[6]]);
        this.addWaypoint(dest[0], dest[1], thetaRad);
        this.createWaypoints();
    }

    viewWaypoints(waypoints) {
        for(let arrow of this.arrow_arr) {
            gridClient.rootObject.removeChild(arrow);
        }
        for(let wp of waypoints) {
            console.log(wp.orientation.x);
            let quat = [wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w]
            let thetaRad = quat2yaw(quat);
            let wp_arrow = new ROS2D.NavigationArrow({
                size : 0.2,
                strokeSize : 0.01,
                fillColor: createjs.Graphics.getRGB(0, 0, 255, 0.9)
            });
            wp_arrow.x = wp.position.x;
            wp_arrow.y = -wp.position.y;
            wp_arrow.rotation = thetaRad * 180.0/Math.PI;

            gridClient.rootObject.addChild(wp_arrow);
            this.arrow_arr.push(wp_arrow);
        }
    }

    clearWaypoints() {
        for(let arrow of this.arrow_arr) {
            gridClient.rootObject.removeChild(arrow);
        }
        this.wp_array = [];
        this.arrow_arr = [];
        let str = "Waypoints 삭제 완료\n";
        status_area.value += str;
        status_area.scrollTop = status_area.scrollHeight;
        let elements = document.getElementsByClassName("marker_info");
        for(let i = 0; i < elements.length; i++) {
            elements.item(i).disabled = false;
        }
    }

    saveWaypoints() {
        let fname = document.getElementById("depart_id").value + '_' + document.getElementById("depart_name").value + '_'
                    + document.getElementById("arrive_id").value + '_' + document.getElementById("arrive_name").value + '.txt';
        let downloadLink = document.createElement("a");
        let textToSave = "";
        for(let wp of this.wp_array) {
            textToSave += String(wp.position.x) + ' '+ String(wp.position.y) + ' ' + String(0.0) + ' '
                       + String(wp.orientation.x) + ' ' + String(wp.orientation.y) + ' ' + String(wp.orientation.z) + ' ' + String(wp.orientation.w) + '\n';
        }
        let textFileAsBlob = new Blob([textToSave], {type:'text/plain'});
        downloadLink.download = fname;
        downloadLink.innerHTML = "Download File";
        if (window.webkitURL != null) {
            // Chrome allows the link to be clicked
            // without actually adding it to the DOM.
            downloadLink.href = window.webkitURL.createObjectURL(textFileAsBlob);
        } else {
            // Firefox requires the link to be added to the DOM
            // before it can be clicked.
            downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
            downloadLink.onclick = destroyClickedElement;
            downloadLink.style.display = "none";
            document.body.appendChild(downloadLink);
        }
        downloadLink.click();

        fname = document.getElementById("arrive_id").value + '_' + document.getElementById("arrive_name").value + '_'
                + document.getElementById("depart_id").value + '_' + document.getElementById("depart_name").value + '.txt';
        
        textToSave = "";
        let wp_array_reverse = this.wp_array;
        wp_array_reverse.reverse();
        for(let i = 0; i < wp_array_reverse.length; i++) {
            let quat = [wp_array_reverse[i].orientation.x, wp_array_reverse[i].orientation.y, 
                    wp_array_reverse[i].orientation.z, wp_array_reverse[i].orientation.w];
            let quat_reverse = quat_mul(quat, yaw2quat(Math.PI));
            let orientation = new ROSLIB.Quaternion({
                x: 0, y: 0, z: quat_reverse[2], w: quat_reverse[3]
            });
            wp_array_reverse[i].orientation = orientation;
        }
        for(let wp of wp_array_reverse) {
            textToSave += String(wp.position.x) + ' '+ String(wp.position.y) + ' ' + String(0.0) + ' '
                       + String(wp.orientation.x) + ' ' + String(wp.orientation.y) + ' ' + String(wp.orientation.z) + ' ' + String(wp.orientation.w) + '\n';
        }
        textFileAsBlob = new Blob([textToSave], {type:'text/plain'});
        downloadLink.download = fname;
        downloadLink.innerHTML = "Download File";
        if (window.webkitURL != null) {
            // Chrome allows the link to be clicked
            // without actually adding it to the DOM.
            downloadLink.href = window.webkitURL.createObjectURL(textFileAsBlob);
        } else {
            // Firefox requires the link to be added to the DOM
            // before it can be clicked.
            downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
            downloadLink.onclick = destroyClickedElement;
            downloadLink.style.display = "none";
            document.body.appendChild(downloadLink);
        }
        downloadLink.click();
    }
}

let waypointHandler = new WaypointHandler();

let wp_check = $("input[id='wpmode']");
let wp_checkbox = document.getElementById("wpmode");
wp_check.click(function() {
    if(wp_checkbox.checked) {
        document.getElementById("wp_cb_status").innerText = "ON";
        let elements = document.getElementsByClassName("marker_info");
        for(let i = 0; i < elements.length; i++) {
            elements.item(i).disabled = false;
        }
    } else {
        document.getElementById("wp_cb_status").innerText = "OFF";
        let elements = document.getElementsByClassName("marker_info");
        for(let i = 0; i < elements.length; i++) {
            elements.item(i).disabled = true;
        }
    }
});

document.getElementById("completebtn").onclick = function() {
    // 바로 onclick = waypointHandler.createWaypoints();를 하면, 정적 메소드가 할당되어 this.wp_array가 undefined가 된다.
    waypointHandler.createWaypoints();
}

document.getElementById("clearbtn").onclick = function () {
    waypointHandler.clearWaypoints();
};

let markerInfo = new Map();

// Parse room information from the file data
function parseMarkerInfo(fileData, point) {
    let arr = fileData.split(' ');
    let xyth = [];
    for(let a of arr) {
        xyth.push(parseFloat(a));
    }
    // Store room information in the Map
    markerInfo.set(point, xyth);
}

document.getElementById('input_file').addEventListener('change', function() {
    let file = new FileReader();

    file.onload = () => {
        let fileData = file.result;
        fileData = fileData.split('\n');

        let startend = fileData[0].split(' ');

        for(let i = 0; i < startend.length/2; i++) {
            let option = document.createElement('option');
            option.value = startend[2*i];
            option.innerText = startend[2*i+1];
            marker.append(option);
        }
        
        // Parse information for each room
        for(let i = 1; i < fileData.length; i++) {
            parseMarkerInfo(fileData[i], startend[2*(i-1)]);
        }
    };
    file.readAsText(this.files[0]);
});

document.getElementById("startbtn").onclick = function() {
    // waypointHandler.createWaypointsByFile();
    let navStartTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/navstart',
        messageType: 'std_msgs/Bool'
    });
    let navStartMessage = new ROSLIB.Message({
        data: true
    });
    navStartTopic.publish(navStartMessage);
};

document.getElementById("savebtn").onclick = function() {
    waypointHandler.saveWaypoints();
};