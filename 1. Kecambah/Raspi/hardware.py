# include library
import asyncio
import websockets
import json
import time
import RPi.GPIO as GPIO 
from datetime import datetime

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# DHT22 pin
DHT22_PIN = 27

# Relay / switching mosfet pin
relay_a = 23
relay_b = 24
relay_c = 25
relay_d = 8

# BTS driver r/lpwm pin
fan1_rpwm_pin = 13  # GPIO pin 13 to the RPWM on the BTS7960
fan1_lpwm_pin = 19  # GPIO pin 19 to the LPWM on the BTS7960
fan2_rpwm_pin = 18
fan2_lpwm_pin = 12

# Enable pin bts driver
fan1_l_en = 6  # connect GPIO pin 6 to L_EN on the BTS7960
fan1_r_en = 6  # connect GPIO pin 6 to R_EN on the BTS7960
fan2_l_en = 5  
fan2_r_en = 5 

inner_count = 0
outer_count = 0
prev_temp = 30
prev_humi = 52

GPIO.setup(relay_a, GPIO.OUT)
GPIO.setup(relay_b, GPIO.OUT)
GPIO.setup(relay_c, GPIO.OUT)
GPIO.setup(relay_d, GPIO.OUT)

GPIO.setup(fan1_rpwm_pin, GPIO.OUT)
GPIO.setup(fan1_l_en, GPIO.OUT)
GPIO.setup(fan1_lpwm_pin, GPIO.OUT)
GPIO.setup(fan1_r_en, GPIO.OUT)
GPIO.setup(fan2_rpwm_pin, GPIO.OUT)
GPIO.setup(fan2_lpwm_pin, GPIO.OUT)
GPIO.setup(fan2_l_en, GPIO.OUT)
GPIO.setup(fan2_r_en, GPIO.OUT)

# Enable L and R driver
GPIO.output(fan1_r_en, True)
GPIO.output(fan1_l_en, True)
GPIO.output(fan2_r_en, True)
GPIO.output(fan2_l_en, True)

# Set frequency for driver
fan1_rpwm = GPIO.PWM(fan1_rpwm_pin, 100)
fan1_lpwm = GPIO.PWM(fan1_lpwm_pin, 100)
fan2_rpwm = GPIO.PWM(fan2_rpwm_pin, 100)
fan2_lpwm = GPIO.PWM(fan2_lpwm_pin, 100)

# Zeroing dc for fan
fan1_rpwm.ChangeDutyCycle(0)
fan1_rpwm.start(0)
fan2_rpwm.ChangeDutyCycle(0)
fan2_rpwm.start(0)

def InitializeStart(dht_pin):
    GPIO.setup(dht_pin, GPIO.OUT)
    GPIO.output(dht_pin, True)
    GPIO.output(dht_pin, False)
    time.sleep(0.019)
    GPIO.output(dht_pin, True)
    GPIO.setup(dht_pin, GPIO.IN)

def ReadDHT(dht_pin):
    pulseTimes = []
    prevState = 1
    timeoutFlag = datetime.now().microsecond
    
    while True:
        if(datetime.now().microsecond - timeoutFlag > 120):
            break
        
        if prevState == 1 and GPIO.input(dht_pin) == 0:
            pulseTimes.append(datetime.now().microsecond-timeoutFlag)
            prevState = 0
            timeoutFlag = datetime.now().microsecond
            
        elif prevState == 0 and GPIO.input(dht_pin) == 1:
            timeoutFlag = datetime.now().microsecond
            prevState = 1
            
    if(len(pulseTimes) == 42):
        pulseBits = TimeListToBits(pulseTimes)
        if(CheckSumController(pulseBits)):
            return pulseBits
        else:
            pass
    else:
        # print("Problem occured while reading sensor")
        pass

def TimeListToBits(TimeList):
    TimeList.pop(0)
    TimeList.pop(0)
    
    BitList = []
    for i in TimeList:
        if i > 45:
            BitList.append(1)
        elif i <= 45:
            BitList.append(0)
            
    return BitList

def CheckSumController(BitList):
    Bin_Byte1 = BitList[0:8]
    Bin_Byte2 = BitList[8:16]
    Bin_Byte3 = BitList[16:24]
    Bin_Byte4 = BitList[24:32]
    Bin_Byte5 = BitList[32:40] #CHECK SUM BYTE!!!
    
    
    dec_byte1 = BinList2Dec(Bin_Byte1)
    dec_byte2 = BinList2Dec(Bin_Byte2)
    dec_byte3 = BinList2Dec(Bin_Byte3)
    dec_byte4 = BinList2Dec(Bin_Byte4)
    dec_byte5 = BinList2Dec(Bin_Byte5)
    
    if(dec_byte1+dec_byte2+dec_byte3+dec_byte4 == dec_byte5):
        # print("CheckSum is correct.")
        return True
    else:
        # print("CheckSum is wrong.")
        return False
    
def BinList2Dec(BitList):
    decimal_data = 0
    for i in BitList:
        decimal_data = (decimal_data << 1) | i
    return decimal_data

def GetHumidity(BitList):
    RH = BitList[0:16]
    decimal_RH = BinList2Dec(RH)
    return decimal_RH/10
    
def GetTemperature(BitList):
    TEMP = BitList[16:32]
    decimal_TEMP = BinList2Dec(TEMP)
    return decimal_TEMP/10

async def read_dht22():
    global prev_temp, prev_humi

    try:
        InitializeStart(DHT22_PIN)
        dht_data = ReadDHT(DHT22_PIN)
        if dht_data:
            temp = GetTemperature(dht_data)
            humi = GetHumidity(dht_data)
            prev_temp = temp
            prev_humi = humi
            return temp, humi
        else:
            print("Use previous data ", end="")
            return prev_temp, prev_humi  # Return None if no data is read
    except Exception as e:
        print(f"Error reading DHT22: {e}")
        return prev_temp, prev_humi

async def control_fan(temperature, threshold, fan1_status, fan2_status):
    global inner_count, outer_count

    if temperature <= threshold:
        inner_count += 1
        outer_count = 0

        if inner_count >= 60:
            inner_count = 0
            if fan2_status == 1:  # Fan 2 is currently on
                # Turn off fan 2
                for dc in range(100, -1, -5):  # Decrease duty cycle
                    fan2_rpwm.ChangeDutyCycle(dc)
                    time.sleep(0.1)
                fan2_status = 0
            if fan1_status == 0:  # Fan 1 is currently off
                # Turn on fan 1
                for dc in range(0, 101, 5):  # Increase duty cycle
                    fan1_rpwm.ChangeDutyCycle(dc)
                    time.sleep(0.1)
                fan1_status = 1
    else:
        inner_count = 0
        outer_count += 1

        if outer_count >= 60:
            outer_count = 0
            if fan1_status == 1:  # Fan 1 is currently on
                # Turn off fan 1
                for dc in range(100, -1, -5):  # Decrease duty cycle
                    fan1_rpwm.ChangeDutyCycle(dc)
                    time.sleep(0.1)
                fan1_status = 0
            if fan2_status == 0:  # Fan 2 is currently off
                # Turn on fan 2
                for dc in range(0, 101, 5):  # Increase duty cycle
                    fan2_rpwm.ChangeDutyCycle(dc)
                    time.sleep(0.1)
                fan2_status = 1
    print(f'Inner c: {inner_count}, Outer c: {outer_count}', end=" ")
    return fan1_status, fan2_status

async def receive_commands(websocket, command_queue):
    while True:
        try:
            message = await websocket.recv()
            command = json.loads(message)
            await command_queue.put(command)  # Add the command to the queue
        except websockets.ConnectionClosed:
            print("Connection closed while receiving commands.")
            break
        except Exception as e:
            print(f"Error receiving command: {e}")

# Handler / main program flow
async def handler(websocket, path):
    fan1_status = 0
    fan2_status = 0
    heater_status = 0
    lamp_status = 0
    uv_status = 0
    threshold_temp = 35
    threshold_humi = 60

    command_queue = asyncio.Queue()  # Create a queue for commands
    # Start the command receiver task
    asyncio.create_task(receive_commands(websocket, command_queue))
    
    while True:
        try:
            # Read DHT22 sensor data
            temp, humi = await read_dht22()
            print(f'Temperature : {temp}    Humidity : {humi}')
            data = {
                'temperature': temp,
                'humidity': humi,
                'fan1_status': fan1_status,
                'fan2_status': fan2_status
            }
            json_data = json.dumps(data)
            await websocket.send(json_data)

            # Control fan behavior
            fan1_status, fan2_status = await control_fan(temp, threshold_temp, fan1_status, fan2_status)

            # Check for commands in the queue
            while not command_queue.empty():
                command = await command_queue.get()
                if "relay_a" in command:
                    GPIO.output(relay_a, command["relay_a"])
                    heater_status = 1 if heater_status == 0 else 0
                    print("heater on" if heater_status == 1 else "heater off")

                if "relay_b" in command:
                    GPIO.output(relay_b, command["relay_b"])
                    lamp_status = 1 if lamp_status == 0 else 0
                    print("lamp on" if lamp_status == 1 else "lamp off")

                if "relay_c" in command:
                    GPIO.output(relay_c, command["relay_c"])
                    uv_status = 1 if uv_status == 0 else 0
                    print("uv on" if uv_status == 1 else "uv off")
                    
                if "relay_d" in command:
                    GPIO.output(relay_d, command["relay_d"])
                    print("relay d on")

                if "target_temp" in command:
                    threshold_temp = command["target_temp"]
                    print("set temp threshold")

                if "target_humi" in command:
                    threshold_humi = command["target_humi"]
                    print("set humi threshold")
                    
                command_queue.task_done()

            await asyncio.sleep(1)  # Send data 0.5 sec

        except websockets.ConnectionClosed:
            print("Client disconnected")
            break
        except Exception as e:
            print(f"Error in handler: {e}")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):  # Listen on all interfaces, port 8765
        print("WebSocket server started on ws://0.0.0.0:8765")
        await asyncio.Future()  # Run forever

asyncio.run(main())
