#!/usr/local/bin/python3
import asyncio
import aioconsole
import platform
import re
import requests
import serial
import struct
from bleak import BleakClient, BleakError

# For establishing bluetooth connection
azalt_uuid = "19b10001-e8f2-537e-4f6c-d104768a1214"
thd_uuid   = "19b10002-e8f2-537e-4f6c-d104768a1214"
dev_uuid   = "C2AC7E0A-F077-4D23-BEFC-23C47C933A6C"

# Globals
pi        = 3.14159
rad       = pi/180.0
deg       = 180.0/pi
brkpt     = 4
brkpt2    = 8
polaris   = [180.4028, 36.8355]
delay     = 60.0
tol_az    = .001
tol_al    = .0005
az_off    = 0.0
al_off    = 0.0

# Position vector
azAlt = {'az' : polaris[0]*rad, 'alt' : polaris[1]*rad}

# Exit
async def exit():
    x = await aioconsole.ainput(">>> ")

# Get temperature, humidity and dewpoint
def getTHD(data):
    print("Getting temperature data...")

    # Get tmp, humidity and dewpoint
    tmp = round(float(struct.unpack('f', data[:brkpt])[0]), 1)
    hum = round(float(struct.unpack('f', data[brkpt:brkpt2])[0]), 1)
    dew = round(float(struct.unpack('f', data[brkpt2:])[0]), 1)
    

    print("Temp: " + str(tmp) + "F    Humidity: " + 
          str(hum) + "%    Dewpoint: " + str(dew) + "F")
          

# Get current az/alt values in degrees
def getAzAltDeg():
    print("Azimuth: " + 
          str(azAlt['az']*deg) + 
          "     Altitude: " +
          str(azAlt['alt']*deg))

# Get current az/alt values in radians
def getAzAltRad():
    print("Azimuth: " + 
          str(azAlt['az']) + 
          "     Altitude: " +
          str(azAlt['alt']))

# Reset offsets
async def calibrate():
    # Offsets
    global az_off
    global al_off

    print("Calculating offsets from current view...")
    r = requests.get("http://localhost:8090/api/objects/info", data={'format':'json'})
    view = {'az' : -r.json()['azimuth']*rad, 'alt' : r.json()['altitude']*rad}

    az_off += view['az'] - azAlt['az']
    al_off += view['alt'] - azAlt['alt']


    print("az_off: " + str(az_off))
    print("al_off: " + str(al_off))

# Function to update Stellarium view via HTTP request
def updateView():
    requests.post("http://localhost:8090/api/main/view", data=azAlt)

# Handles updates to azAlt data
def notification_handler(sender, data):
    # To update or not
    update = False

    # Get az and alt and convert to radians

    az  = round((float(struct.unpack('f', data[:brkpt])[0]) - 360.0)*rad + az_off, 4)
    alt = round(float(struct.unpack('f', data[brkpt:])[0])*rad + al_off, 4)

    # Check if an update is necessary
    if abs(azAlt['az'] - az) > tol_az:
        # Check for overflow from offset addition
        if az > 0 or az < -2.0*pi:
            az = -(-az % (2.0*pi))
        azAlt['az'] = az
        update = True

    if abs(azAlt['alt'] - alt) > tol_al:
        # Check for overflow from offset addition
        if alt > pi/2.0:
            alt -= 2*(alt%(pi/2.0))
            azAlt['az'] -= pi
            if azAlt['az'] < -2.0*pi:
                azAlt['az'] = -(-azAlt['az'] % (2.0*pi))
        azAlt['alt'] = alt
        update = True

    # Update Stellarium view
    if update:
        updateView()
    

# Establish BLE connection and loop continuously
async def get_azAlt(dev_uuid: str, loop: asyncio.AbstractEventLoop):
    async with BleakClient(dev_uuid, loop=loop) as client:
        # Ensure connection is established
        await client.is_connected()
        print("Client Connected!")

        # Enable notification and assign handler
        await client.start_notify(azalt_uuid, notification_handler)

        # Wait for notifications
        while True:
            # Await user input
            x = await aioconsole.ainput(">>> ")
            x = x.lower()

            if x == 'c' or x == "calibrate":
                await calibrate()

            if x == 't' or x == "temperature":
                tmp = await client.read_gatt_char(thd_uuid)
                getTHD(tmp)

            if x == 'g' or x == "get az/alt":
                selection = await aioconsole.ainput("Radians or Degrees (R/D)?: ")
                if selection == 'R' or selection == 'r':
                    getAzAltRad()
                else:
                    getAzAltDeg()


try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(get_azAlt(dev_uuid, loop))
except KeyboardInterrupt:
    loop.stop()
    print("\nExiting...")
except BleakError as e:
    loop.stop()
    print("\n" + str(e) + "\n")
except requests.exceptions.RequestException as e:
    loop.stop()
    print("\n" + str(e) + "\n")
