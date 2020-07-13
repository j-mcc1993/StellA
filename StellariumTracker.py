import serial, requests

# Locals
pi    = 3.14159
rad   = pi/180.0
tol   = 0.01

# Position vector
azAlt = {'az' : pi, 'alt' : 0.0}

# Function to update Stellarium view via HTTP request
def updateView():
    requests.post("http://localhost:8090/api/main/view", data=azAlt)

# Initialize serial connection with Arduino
arduino = serial.Serial('/dev/cu.usbmodem14201', 9600, timeout=.1)

# Initialize Stellarium view
updateView()

# Loop to read Arduino sensor and update view 
while True:
    data = arduino.readline()[:-2] # remove newlines
    if data:
        data = data.decode('utf8')
        print(data)

        data = [float(x)*rad for x in data.split()]
        if len(data) != 2:
            print("Error! Incomplete line of data read")
            continue
        
        # If full line of data was read, obtain alt/az values
        delt_az  = abs(azAlt['az'] - data[0])
        delt_alt = abs(azAlt['alt'] - data[1])

        if delt_az > tol or delt_alt > tol:
            azAlt['az']  = data[0]
            azAlt['alt'] = data[1]
            updateView()
