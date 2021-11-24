// Global device and characteristic objects
var device = null;
var downlinkCharacteristic = null;
var uplinkCharacteristic = null;

// UUIDs for services and characteristics
var siliconwitcheryUUID = "48fd0481-184d-fe9e-e84d-bfc537115aab";
var cameraService = "48fd1000-184d-fe9e-e84d-bfc537115aab";
var cameraCharacteristic = "48fd1001-184d-fe9e-e84d-bfc537115aab";

// Promise function to check if bluetooth is availabe in the browser
function isWebBluetoothAvailable() {
    return new Promise((resolve, reject) => {
        navigator.bluetooth ? resolve() : reject("BLE not available");
    });
}

// Function to connect and disconnect, returning status as promise
async function connectDisconnect() {
    try {
        // First ensure web bluetooth is available
        await isWebBluetoothAvailable();

        // Disconnect if connected
        if (device) {
            if (device.gatt.connected) {
                await device.gatt.disconnect();
                return Promise.resolve("disconnected");
            }
        }

        // Otherwise connect
        device = await navigator.bluetooth.requestDevice({
            filters: [{
                // namePrefix: "S1 Thermal camera demo",
                services: [cameraService]
            }]
        });

        // Handler to watch for device being disconnected due to loss of connection
        device.addEventListener('gattserverdisconnected', disconnectHandler);

        // Connect to device and get primary service as well as characteristics
        const server = await device.gatt.connect();
        const service = await server.getPrimaryService(cameraService);
        cameraDataEndpoint = await service.getCharacteristic(cameraCharacteristic);

        // Start notifications on receving characteristic and crete handler
        await cameraDataEndpoint.startNotifications();
        cameraDataEndpoint.addEventListener('characteristicvaluechanged',
            dataNotificationHandler);

        // Connected as unsecure method
        return Promise.resolve("unsecure");

    } catch (error) {
        // Return error if there is any
        return Promise.reject(error);
    }
}

// Raw pixel data which comes in over bluetooth as uint8 bytes
var rawDataArray = [];

// The float converted pixels are stored in here
var finalPixelArray = [];

// Callback handling received data from bluetooth
function dataNotificationHandler(event) {

    // If the first byte is 1, then we reset the byte counter for a new frame
    if (event.target.value.getUint8(0) == 0) {
        rawDataArray = [];
    }

    // While the array counter is less than the frame size, we copy data over
    for (let i = 0; i < event.target.value.byteLength - 1; i++) {
        rawDataArray.push(event.target.value.getUint8(1 + i))
    }

    // Once we get the first byte flag as 2, then we process the data
    if (event.target.value.getUint8(0) == 2) {

        // Create data view from the array
        var uint8data = new DataView(new ArrayBuffer(rawDataArray.length));

        // Set bytes
        rawDataArray.forEach(function (b, i) {
            uint8data.setUint8(i, b);
        });

        // Convert the raw data to floats 24*32 pixels is 768
        for (let i = 0; i < 768; i++) {
            // For every four bytes, copy uint8 as float
            finalPixelArray[i] = uint8data.getFloat32(i * 4);
        }

        updatePixelGrid(finalPixelArray);
    }
}