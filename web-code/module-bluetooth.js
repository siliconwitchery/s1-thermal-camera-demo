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

// Callback handling received data from bluetooth
function dataNotificationHandler(event) {

    // Decode the byte array into a string
    const decoder = new TextDecoder('utf-8');
    let value = event.target.value;
    console.log(value);

    // TODO parse the data to the pixel array
}