const statusDiv = document.getElementById('status');
const dataDisplayDiv = document.getElementById('data-display');
const leftZone = document.getElementById('joystick-left');
const rightZone = document.getElementById('joystick-right');

let websocket = null;
let leftStickY = 0.0;
let rightStickX = 0.0;
let sendInterval = null; // To periodically send data

// --- Joystick Configuration ---
const joystickOptions = {
    mode: 'static', // Keep joystick centered in its zone
    color: 'blue',
    size: 100, // Size of the nipple
    threshold: 0.1, // Minimum movement to trigger event
    fadeTime: 250,
    multitouch: true,
    maxNumberOfNipples: 2,
    dataOnly: false,
    position: { top: '50%', left: '50%' }, // Center it in the zone
    restJoystick: true,
    restOpacity: 0.5,
    lockX: false, // Default: allow X and Y
    lockY: false,
};

// --- Create Left Joystick (Only Y matters) ---
const leftManager = nipplejs.create({
    ...joystickOptions, // Spread common options
    zone: leftZone,
    color: 'green',
    lockX: true, // Lock X-axis movement
});

leftManager.on('move', (evt, data) => {
    if (data.vector) {
        // nipplejs gives vector.y positive downwards, ROS usually expects positive forwards
        // Normalize distance to range [-1, 1] based on joystick size/2
        leftStickY = -data.vector.y; // Invert Y
    }
    updateDataDisplay();
});

leftManager.on('end', () => {
    leftStickY = 0.0;
    updateDataDisplay();
    sendJoystickData(); // Send zero value immediately on release
});

// --- Create Right Joystick (Only X matters) ---
const rightManager = nipplejs.create({
    ...joystickOptions, // Spread common options
    zone: rightZone,
    color: 'red',
    lockY: true, // Lock Y-axis movement
});

rightManager.on('move', (evt, data) => {
    if (data.vector) {
        // nipplejs gives vector.x positive rightwards, ROS twist often uses positive anti-clockwise
        // We'll keep positive right for now, mapping happens in ROS node if needed
        rightStickX = data.vector.x;
    }
    updateDataDisplay();
});

rightManager.on('end', () => {
    rightStickX = 0.0;
    updateDataDisplay();
    sendJoystickData(); // Send zero value immediately on release
});

// --- WebSocket Connection ---
function connectWebSocket() {
    // Construct WebSocket URL (adjust hostname/port if needed)
    const wsUrl = `ws://${window.location.hostname}:9090`; // Use port 9090
    statusDiv.textContent = `Status: Connecting to ${wsUrl}...`;
    websocket = new WebSocket(wsUrl);

    websocket.onopen = () => {
        statusDiv.textContent = 'Status: Connected';
        console.log('WebSocket Connected');
        // Start sending data periodically
        if (sendInterval) clearInterval(sendInterval);
        sendInterval = setInterval(sendJoystickData, 100); // Send data every 100ms
    };

    websocket.onclose = (event) => {
        statusDiv.textContent = `Status: Disconnected (${event.code})`;
        console.log('WebSocket Disconnected:', event.reason);
        websocket = null;
        if (sendInterval) clearInterval(sendInterval);
        sendInterval = null;
        // Optional: Try to reconnect after a delay
        setTimeout(connectWebSocket, 5000); // Reconnect after 5 seconds
    };

    websocket.onerror = (error) => {
        statusDiv.textContent = 'Status: Error';
        console.error('WebSocket Error:', error);
        // The onclose event will likely fire next, triggering reconnection attempt
    };

    // We don't expect messages from the server in this simple example
    // websocket.onmessage = (event) => {
    //     console.log('Message from server:', event.data);
    // };
}

// --- Send Data ---
function sendJoystickData() {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        const data = {
            left_y: leftStickY.toFixed(3), // Send as strings for precision maybe?
            right_x: rightStickX.toFixed(3),
        };
        try {
            websocket.send(JSON.stringify(data));
            // console.log('Sent:', data);
        } catch (error) {
            console.error("Failed to send data:", error);
        }
    }
}

// --- Update Display ---
function updateDataDisplay() {
    dataDisplayDiv.textContent = `Left Y: ${leftStickY.toFixed(2)}, Right X: ${rightStickX.toFixed(2)}`;
}

// --- Initial Connection ---
connectWebSocket();