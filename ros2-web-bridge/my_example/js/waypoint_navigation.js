var wp_check = $("input[id='wpmode']");
var wp_checkbox = document.getElementById("wpmode");
wp_check.click(function() {
    wp_checkbox.checked? document.getElementById("wp_cb_status").innerText = "ON": document.getElementById("wp_cb_status").innerText = "OFF";
    console.log("waypoint")
});