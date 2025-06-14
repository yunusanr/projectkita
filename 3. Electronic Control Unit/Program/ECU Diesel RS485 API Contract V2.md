# ECU Diesel RS485 API Contract

## Protocol
- **Protocol**: RS485

## Communication Settings
- **Baud Rate**: 9600
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Timeout (ms)**: 1000

## Commands

### 1. Voltage Sensor
- **Address**: 0x40000 - Register data
- **Description**: Read data from voltage sensor
- **Type Data**: Int
- **Status Address**: 
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: Voltage
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: 12
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 2. Current Sensor 
- **Address**: 0x40001 - Register data
- **Description**: Read data from current sensor (INA219)
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: Ampere
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: -
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 3. Temperature (1)
- **Address**: 0x40002 - Register data
- **Description**: Read from thermocouple 1
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: Celcius
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: -
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 4 Temperature (2)
- **Address**: 0x40003 - Register data
- **Description**: Read from thermocouple 2
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: Celcius
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: -
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 5 RPM
- **Address**: 0x40004- Register data
- **Description**: Read from RPM sensor (proximity sensor)
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: rpm
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: -
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 6 fuel
- **Address**: 0x40005  Register data
- **Description**: Read from fuel sensor
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: percentage
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: 100
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 7 odo hours meter
- **Address**: 0x40006  Register data
- **Description**: Read from last value from odoH
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: Seconds
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: -
- **Save to EEPROOM**: yes
- **Update Interval**: 5 Second

### 8 current odo hours meter
- **Address**: 0x40007  Register data
- **Description**: Read from current value from odoH
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Read only
- **Unit Value**: Seconds
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: -
- **Save to EEPROOM**: yes
- **Update Interval**: 5 Second

### 9 start function
- **Address**: 0x40008 Register data
- **Description**: Function to start the machine
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Write only
- **Unit Value**: 
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: 1
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 10 stop function
- **Address**: 0x40009 register data
- **Description**: Function to stop the machine
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Write only
- **Unit Value**: 
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: 1
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 11 Servo percentage
- **Address**: 0x40010 Register data
- **Description**: Function to send Servo value
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Write only
- **Unit Value**: Percentage
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: 100
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second

### 12 Servo move
- **Address**: 0x40011 Register data
- **Description**: Function to move servo based on Servo percentage
- **Type Data**: Int
- **Status Address**:
- **Parity & Stop Bit**: 8N1
- **Status Data**: Write only
- **Unit Value**: 
- **Default Value**: 0
- **Min Value**: 0
- **Max Value**: 1
- **Save to EEPROOM**: no
- **Update Interval**: 5 Second
- 
## Error Handler
- **Address**: 0x00xxxx
- **Description**: Error failure status
- **Type Data**: Read Only
- **Status Address**: (Misal panjang data 2, starting address 0, Error code misa 01, 02 , 03 dst - Detail Error Message di bawah)
- **Parity & Stop Bit**: (Misal 8N1)
- **Status Data**: Read only/ Write Only / Read and Write
- **Unit Value**: Code Integer
- **Default Value**: 0
- **Min Value**: 1
- **Max Value**: 100
- **Save to EEPROOM**: yes/no
- **Update Interval**: Accidental

### Common Error Codes
- `01`: Illegal Function
- `02`: Illegal Data Address
- `03`: Illegal Data Value
- `04`: Slave Device Failure
