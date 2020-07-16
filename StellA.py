import asyncio
import platform
import requests
import serial
import struct
from bleak import BleakClient, BleakError

# For establishing bluetooth connection
char_uuid = "19b10001-e8f2-537e-4f6c-d104768a1214"
dev_uuid  = "E5DD24C3-3942-453F-91E5-5568D1BB4908"

# Globals
pi    = 3.14159
rad   = pi/180.0
brkpt = 4
north = 37.385050 
delay = 60.0

# Position vector
azAlt = {'az' : pi, 'alt' : north}

# Function to update Stellarium view via HTTP request
def updateView():
    requests.post("http://localhost:8090/api/main/view", data=azAlt)

# Handles updates to azAlt data
def notification_handler(sender, data):
    az  = float(struct.unpack('f', data[:brkpt])[0])
    alt = float(struct.unpack('f', data[brkpt:])[0])
    print("az: " + str(round(az,1)) + "    alt: " + str(round(alt,1)))

    # Convert to radians
    az  *= rad
    alt *= rad
    azAlt['az']  = az
    azAlt['alt'] = alt

    # Update Stellarium view
    updateView()
    

# Establish BLE connection and loop continuously
async def get_azAlt(dev_uuid: str, loop: asyncio.AbstractEventLoop):
    async with BleakClient(dev_uuid, loop=loop) as client:
        # Ensure connection is established
        await client.is_connected()
        
        # Enable notification and assign handler
        await client.start_notify(char_uuid, notification_handler)

        # Wait for notifications
        while True:
            await asyncio.sleep(delay)


try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(get_azAlt(dev_uuid, loop))
except KeyboardInterrupt:
    loop.stop()
    print("\nExiting...")
except BleakError:
    loop.stop()
    print("\nDevice couldn't be found, please reset and try again.\n")
