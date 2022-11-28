// 방 정보가 저장된 파일 불러오기
let rooms_info = new Map();
let rooms = document.getElementById("room_id");
document.getElementById('input_file').addEventListener('change', function() {
    let parseRoomsInfo = (file_data) => {
        let arr = file_data.split('"');
        let xyth = arr[2].split(' ');
        rooms_info.set(
            arr[0],
            [[parseFloat(xyth[1]), parseFloat(xyth[2]), parseFloat(xyth[3])],
            [parseFloat(xyth[4]), parseFloat(xyth[5]), parseFloat(xyth[6])]]
        );
        let option = document.createElement('option');
        option.value = arr[0];
        option.innerText = arr[1];
        rooms.append(option);
    }

    let file = new FileReader();
    file.onload = () => {
        let file_data = file.result;
        file_data = file_data.split('\n');
        for(var i = 0; i < file_data.length - 1; i++) {
            parseRoomsInfo(file_data[i]);
        }
    };
    file.readAsText(this.files[0]);
});

let nav_mode = new ROSLIB.Topic({
    ros: ros,
    name: "tracking_mode",
    messageType: "std_msgs/String"
});

let head_nav = false, tail_nav = "tracking";
function navigation_pipeline() {
    let dest = rooms_info.get(document.getElementById("room_id").value);
    head_drive(dest[1][0], dest[1][1], dest[1][2]); // head_point 이동
    tail_drive(dest[0][0], dest[0][1], dest[0][2]); // 각 방에 전달
    tail_nav = "tracking";
}

document.getElementById("startbtn").onclick = navigation_pipeline;

// navigation 동작
function head_drive(x, y, thetaRad) {
    head_nav = true;
    createGoalPose(x, y, thetaRad);
    // 도착했음을 나타내기 전까지 대기하도록 수정
    head_nav = false;
    tail_nav = "navigating";
    console.log("Arrive head point!");
};

function head_comeback() {

};

function tail_drive(x, y, thetaRad) {
    createGoalPose(x, y, thetaRad);
    // tail이 도착했음을 head에 전달(수취확인시)
    console.log("Arrive room point!");
};

function tail_comeback() {

};