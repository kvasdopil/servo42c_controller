/**
 * Modbus RTU Motor Control Script
 * -----------------------------
 * This script connects to a motor controller via a serial port (USB adapter)
 * using the Modbus RTU protocol.
 *
 * It performs the following actions in a continuous loop:
 * 1. Sends an absolute move command to position 1 (e.g., 100 pulses).
 * 2. Monitors and prints the current motor position for a set duration.
 * 3. Sends an absolute move command to position 2 (e.g., 0 pulses).
 * 4. Monitors and prints the current motor position for a set duration.
 * 5. Repeats the cycle.
 *
 * Configuration constants (SERIAL_PORT, BAUD_RATE, TARGET_SLAVE_ID, etc.)
 * should be adjusted below to match the specific hardware setup.
 *
 * Press Ctrl+C to exit gracefully.
 */
'use strict';

const ModbusRTU = require("modbus-serial");

// --- Configuration ---
const SERIAL_PORT = "/dev/ttyUSB0"; // Adapter path
const BAUD_RATE = 38400;         // Baud rate (User confirmed)
const TARGET_SLAVE_ID = 2;       // Device ID
const TIMEOUT = 1000;            // Timeout (ms)

// Read Addresses
const ENC_VAL_ADDR = 48;
const ENC_VAL_QTY = 3;
const POS_ADDR = 51;
const POS_QTY = 2;

// Write Addresses
const MOVE_ABS_ADDR = 254;

// Loop Settings
const MOVE_ACCEL = 0xf0; // Minimal sensible acceleration
const MOVE_SPEED = 0xff; // RPM for moves
const POSITION_1 = 30000;
const POSITION_2 = 0;
const SLEEP_DURATION_MS = 5000; // 5 seconds
const MONITOR_INTERVAL_MS = 500; // Read position every 500ms during sleep
// ---------------------

const client = new ModbusRTU();

// --- Utility Functions ---
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// --- Read Functions (Assuming buffer contains only data bytes) ---
async function readPosition(client) {
    try {
        const response = await client.readInputRegisters(POS_ADDR, POS_QTY);
        if (response && response.buffer && response.buffer.length === 4) {
            const dataPayload = response.buffer;
            const pulses = dataPayload.readUInt32BE(0);
            return pulses; // Return the value
        } else {
            // Restore error logging for unexpected format
            console.error("\n -> Position Read Error: Unexpected response format. Buffer:", response ? response.buffer : 'N/A');
            return null;
        }
    } catch (err) {
        // Restore error logging, but be less verbose for common timeouts during monitoring
        if (!err.message || (!err.message.includes("Timed out") && !err.message.includes("Port Not Open"))) {
            console.error(`\n -> Position Read Error: ${err.message || err}`);
            if (err.modbusCode) console.error(`   Modbus Exception: ${err.modbusCode}`);
        } else {
            // Log timeouts less intrusively during monitoring
            process.stdout.write('\rPosition Read Timeout...        ');
        }
        return null;
    }
}

// --- Write Function ---
async function moveAbsolute(client, acceleration, speed, absPulses) {
    console.log(`\n>>> Sending Absolute Move Command: Accel=${acceleration}, Speed=${speed}, TargetPos=${absPulses}`); // Restore log
    const address = MOVE_ABS_ADDR;
    const register1 = acceleration & 0xFFFF;
    const register2 = speed & 0xFFFF;
    const register3 = (absPulses >> 16) & 0xFFFF;
    const register4 = absPulses & 0xFFFF;
    const dataArray = [register1, register2, register3, register4];

    try {
        const response = await client.writeRegisters(address, dataArray);
        if (response && response.address === address && response.length === dataArray.length) {
            console.log(" -> Move command acknowledged."); // Restore log
            return true;
        } else {
            // Restore error logging
            console.error(" -> Move command failed: Unexpected response. Response:", response);
            return false;
        }
    } catch (err) {
        // Restore error logging
        console.error(`\n -> Move command failed: ${err.message || err}`);
        if (err.modbusCode) console.error(`   Modbus Exception: ${err.modbusCode}`);
        return false;
    }
}

/**
 * Reads the current position periodically for a set duration.
 * @param {ModbusRTU} client The connected client
 * @param {number} durationMs Total time to monitor
 * @param {number} intervalMs How often to read position
 */
async function monitorPosition(client, durationMs, intervalMs) {
    const endTime = Date.now() + durationMs;
    console.log(`--- Monitoring position for ${durationMs / 1000}s ---`); // Restore log
    while (Date.now() < endTime) {
        const currentPos = await readPosition(client);
        // Restore position printing
        if (currentPos !== null) {
            process.stdout.write(`\rCurrent Position: ${currentPos}   `);
        } else {
            // The readPosition function now handles timeout logging less intrusively
            // We might not need this else block unless readPosition returns null for other reasons
            // process.stdout.write(`\rPosition Read Error...        `);
        }
        const remainingTime = endTime - Date.now();
        const sleepTime = Math.min(intervalMs, remainingTime > 0 ? remainingTime : 0);
        if (sleepTime > 0) {
            await sleep(sleepTime);
        }
    }
    process.stdout.write("\r                                     \r"); // Clear line
    console.log(`--- Monitoring finished ---`); // Restore log
}


/**
 * Main application loop.
 */
async function main() {
    let isConnected = false;
    try {
        console.log(`Connecting to ${SERIAL_PORT} at ${BAUD_RATE} baud...`); // Restore log
        await client.connectRTUBuffered(SERIAL_PORT, { baudRate: BAUD_RATE });
        isConnected = true;
        console.log("Connected. Setting up client..."); // Restore log
        client.setID(TARGET_SLAVE_ID);
        client.setTimeout(TIMEOUT);

        console.log("\nStarting position cycling loop (Ctrl+C to exit)..."); // Restore log

        while (true) {
            if (!(await moveAbsolute(client, MOVE_ACCEL, MOVE_SPEED, POSITION_1))) {
                console.error("Aborting cycle due to move error."); // Restore log
                break;
            }
            await monitorPosition(client, SLEEP_DURATION_MS, MONITOR_INTERVAL_MS);

            if (!(await moveAbsolute(client, MOVE_ACCEL, MOVE_SPEED, POSITION_2))) {
                console.error("Aborting cycle due to move error."); // Restore log
                break;
            }
            await monitorPosition(client, SLEEP_DURATION_MS, MONITOR_INTERVAL_MS);

        }

    } catch (err) {
        console.error("\n--- MAIN ERROR ---");
        console.error("An unexpected error occurred:", err.message || err);
        if (err.stack) console.error(err.stack);
        console.error("------------------\n");
    } finally {
        if (client.isOpen) {
            console.log("\nClosing connection..."); // Restore log
            await sleep(100);
            client.close(() => {
                console.log("Connection closed."); // Restore log
            });
        } else if (isConnected) {
            console.log("\nConnection already closed."); // Restore log
        } else {
            console.log("\nClient was not connected."); // Restore log
        }
    }
}

// --- Run the main function ---
main();

// Graceful exit handling
process.on('SIGINT', () => {
    console.log('\nCaught interrupt signal (Ctrl+C). Exiting...');
    if (client.isOpen) {
        client.close(() => {
            console.log("Connection closed due to exit signal.");
            process.exit(0);
        });
    } else {
        process.exit(0);
    }
    setTimeout(() => {
        console.log("Forcing exit.");
        process.exit(1);
    }, 2000);
}); 