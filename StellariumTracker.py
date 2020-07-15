import asyncio
import platform
import requests
import serial
import struct
from bleak import BleakClient, BleakError

# For establishing bluetooth connection
char_uuid = "19b10001-e8f2-537e-4f6c-d104768a1214"
mac_addr = "E5DD24C3-3942-453F-91E5-5568D1BB4908"

# Globals
pi  = 3.14159
rad = pi/180.0
bp  = 4

# Position vector
azAlt = {'az' : pi, 'alt' : 0.0}

# Function to update Stellarium view via HTTP request
def updateView():
    requests.post("http://localhost:8090/api/main/view", data=azAlt)

# Handles updates to azAlt data
def notification_handler(sender, data):
    az  = float(struct.unpack('f', data[:bp])[0])
    alt = float(struct.unpack('f', data[bp:])[0])
    print("az: " + str(az) + "    alt: " + str(alt))

    # Convert to radians
    az  *= rad
    alt *= rad
    azAlt['az']  = az
    azAlt['alt'] = alt

    # Update Stellarium view
    updateView()
    

# Establish BLE connection and loop continuously
async def get_azAlt(mac_addr: str, loop: asyncio.AbstractEventLoop):
    async with BleakClient(mac_addr, loop=loop) as client:
        # Ensure connection is established
        x = await client.is_connected()
        
        # Enable notification and assign handler
        await client.start_notify(char_uuid, notification_handler)

        # Wait for notifications
        while True:
            await asyncio.sleep(15.0, loop=loop)


try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(get_azAlt(mac_addr, loop))
    loop.run_forever()
except KeyboardInterrupt:
    loop.stop()
    print("\nExiting...")
except BleakError:
    loop.stop()
    print("\nDevice couldn't be found, please reset and try again\n")
