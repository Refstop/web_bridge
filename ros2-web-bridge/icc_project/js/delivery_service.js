// 방 정보가 저장된 파일 불러오기
// 호수, 방 이름, 방 앞(x, y, theta), headpoint(x, y, theta)
let roomsInfo = new Map();
let rooms = document.getElementById("room_id");

// Parse room information from the file data
function parseRoomsInfo(fileData) {
    let arr = fileData.split('"');
    let xyth = arr[2].split(' ');

    // Store room information in the Map
    roomsInfo.set(
        arr[0],
        [
            [parseFloat(xyth[1]), parseFloat(xyth[2]), parseFloat(xyth[3])],
            [parseFloat(xyth[4]), parseFloat(xyth[5]), parseFloat(xyth[6])]
        ]
    );

    // Create an option element for the room
    let option = document.createElement('option');
    option.value = arr[0];
    option.innerText = arr[1];
    rooms.append(option);
}

document.getElementById('input_file').addEventListener('change', function() {
    let file = new FileReader();

    file.onload = () => {
        let fileData = file.result;
        fileData = fileData.split('\n');
        
        // Parse information for each room
        for(let i = 0; i < fileData.length - 1; i++) {
            parseRoomsInfo(fileData[i]);
        }
    };
    file.readAsText(this.files[0]);
});

let nav_mode = new ROSLIB.Topic({
    ros: ros,
    name: "tracking_mode",
    messageType: "std_msgs/String"
});

const STATUS = {
    WAITING : 0, // 대기
    DEPART_BACK : 1, // 복귀
    HEAD_POINT : 2, // Head Point 도착
    ROOM_POINT : 3, // Room Point 도착

    TRACKING : 4, // 마커 추적 모드
    NAVIGATING : 5 // navigating 모드
};

// state들과 tail_mode의 변화는 head/tail_arrive, reception(수취확인) subscribe 함수 안에서만 발생
// createGoalPose: tail_mode가 tracking-omo/goal_pose, navigating-그냥 goal_pose를 publish
let head_state = STATUS.WAITING, tail_state = STATUS.WAITING;
let tail_mode = STATUS.TRACKING;
function navigation_pipeline() {
    let dest = roomsInfo.get(document.getElementById("room_id").value);
    let str = "";
    while(1) {
        if(head_state == STATUS.WAITING && tail_mode == STATUS.TRACKING) {
            // 1. head_point 이동
            createGoalPose(dest[1][0], dest[1][1], dest[1][2]);
        } else if(head_state == STATUS.HEAD_POINT && tail_mode == STATUS.TRACKING) { 
            // 2. head_point에 도착하면
            str = "1차 경유 지점 도착\n";
            status_area.value += str;
            status_area.scrollTop = status_area.scrollHeight;
            createGoalPose(dest[0][0], dest[0][1], dest[0][2]); // 3. tail을 보낸다
        } else if(head_state == STATUS.DEPART_BACK && tail_mode == STATUS.TRACKING) {
            let home = roomsInfo.get("000 ");
            createGoalPose(home[0][0], home[0][1], home[0][2]);
            break;
        } else if(tail_state == STATUS.ROOM_POINT && tail_mode == STATUS.NAVIGATING) {
            if(str != "배송지 도착\n") {
                str = "배송지 도착\n";
                status_area.value += str;
                status_area.scrollTop = status_area.scrollHeight;
            } else {
                tail_state = STATUS.DEPART_BACK;
            }
        } else if(tail_state == STATUS.DEPART_BACK && tail_mode == STATUS.NAVIGATING) {
            createGoalPose(x, y, theta);
        }
    }
}
document.getElementById("startbtn").onclick = navigation_pipeline;

let head_arrive_topic = new ROSLIB.Topic({
    ros: ros,
    name: head_namespace + "/follow_path/_action/status",
    messageType: "action_msgs/GoalStatusArray"
});
head_arrive_topic.subscribe((msg)=> {
    // int8 STATUS_SUCCEEDED = 4
    if(msg.status_list[msg.status_list.length - 1].status == 4 && tail_mode == STATUS.TRACKING) {
        if(head_state == STATUS.WAITING) {
            head_state = STATUS.HEAD_POINT;
            tail_mode = STATUS.NAVIGATING;
        } else if(head_state == STATUS.HEAD_POINT) {
            head_state = STATUS.DEPART_BACK;
        }
    }
});

let tail_arrive_topic = new ROSLIB.Topic({
    ros: ros,
    name: tail_namespace + "/follow_path/_action/status",
    messageType: "action_msgs/GoalStatusArray"
});
tail_arrive_topic.subscribe((msg)=> {
    // int8 STATUS_SUCCEEDED = 4
    if(msg.status_list[msg.status_list.length - 1].status == 4) {
        if(tail_state == STATUS.DEPART_BACK) {
            tail_state = STATUS.WAITING;
            tail_mode = STATUS.TRACKING;
            head_state = STATUS.DEPART_BACK;
        } else if(tail_state == STATUS.WAITING) {
            tail_state = STATUS.ROOM_POINT;
        }
    }
});

let reception_topic = new ROSLIB.Topic({
    ros: ros,
    name: tail_namespace + "/reception",
    messageType: "std_msgs/Empty"
});
reception_topic.subscribe((msg)=> {
    // tail_state를 DEPART_BACK으로 바꿔주는건 여기뿐이다
    tail_state == STATUS.DEPART_BACK
});