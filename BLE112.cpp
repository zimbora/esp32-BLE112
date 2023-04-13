
#include "BLE112.h"

extern HardwareSerial *ble;
uint8_t command[MAX_PACKET_SIZE+HEADER_SIZE];
uint8_t response[MAX_PACKET_SIZE+HEADER_SIZE];
uint8_t response_size = 0;

uint8_t inc = 0;
uint8_t gpio_hw_reset = -1;
packet msg;
uint16_t connection = 0;
uint8_t i = 0;
struct Module module{
  0,
  0,
  0,
  0,
  0,
  0,
  0
};

struct Scan scan;

struct services service[MAX_SERVICES];
struct characteristics char_[MAX_CHARACTERISTICS];
struct descriptors desc_[MAX_DESCRIPTORS];
uint8_t s_index = 0;
uint8_t c_index = 0;

BLE112::BLE112(uint32_t baudrate,uint8_t gpio){
  gpio_hw_reset = gpio;
  ble->begin(baudrate,SERIAL_8N1,27,14);
  pinMode(gpio_hw_reset,OUTPUT);
	digitalWrite(gpio_hw_reset, HIGH);
}

bool BLE112::initModule(){
  uint32_t timeout = millis()+5000;

  while(!hello() && timeout > millis()){
    delay(2000);
    hardwareReset();
  }

  slave.battery = 0;
  slave.rssi = 0xFF;
  slave.connected = false;

  if(timeout < millis())
    return false;
  else return true;
}

bool BLE112::set_callback(void(*parseEvents)(uint16_t,uint8_t*,uint8_t)){
  if(parse_rcv_data) return false; // allready defined

  parse_rcv_data = parseEvents; // define it

  return true;
}

bool BLE112::findMac(uint8_t* mac, uint32_t timeout){
  timeout += millis();
  scan.mac_found = false;
  Serial.print("discover mac: ");
  PrintHex8(mac,6);
  memcpy(scan.discover,mac,6);
  discover(1);
  while(timeout > millis() && !scan.mac_found){
    checkMessages();
  }
  endProcedure(1);
  return scan.mac_found;
}

bool BLE112::findMac(uint8_t* mac, uint32_t timeout, uint8_t mode){
  timeout += millis();
  scan.mac_found = false;
  Serial.print("discover mac: ");
  PrintHex8(mac,6);
  memcpy(scan.discover,mac,6);
  discover(mode);
  while(timeout > millis() && !scan.mac_found){
    checkMessages();
  }
  endProcedure(mode);
  return scan.mac_found;
}


uint8_t BLE112::softwareReset(){
  msg.length = HEADER_SIZE+1;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = 0x09; // FW upgrade
  msg.cmd = 0x00;
  msg.data[0] = 0;

  sendCommand(WAIT_RESPONSE,1000);

  return true;
}

void BLE112::hardwareReset(){
  //Disable BLE module power -> bit6: 0
	digitalWrite(gpio_hw_reset, LOW);

  delay(500);

  //Enable BLE module power -> bit6: 1
  digitalWrite(gpio_hw_reset, HIGH);

  delay(1000);

}

void BLE112::checkMessages(){
  readCommand(0);
  return;
}

bool BLE112::hello(){

  msg.length = HEADER_SIZE;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_SYSTEM;
  msg.cmd = 0x01; // Message ID

  return sendCommand(WAIT_RESPONSE,1000);
}

void BLE112::list_characteristics(){
  //#ifdef BLE_DEBUG
  uint8_t i = 0;
  while(i<c_index){
    Serial.printf("type: %d \n",char_[i].type);
    Serial.printf("permissions: %d \n",char_[i].permissions);
    Serial.printf("handle: %d \n",char_[i].handle);
    Serial.printf("uuid: %x %x \n",(uint8_t)(char_[i].uuid>>8),(uint8_t)char_[i].uuid);
    i++;
  }
  //#endif
}

void BLE112::get_all_characteristics(){
  uint8_t i = 0;
  while(i<MAX_SERVICES){
    #ifdef BLE_DEBUG
    Serial.print("service: ");
    PrintHex8(service[i].id,service[i].len);
    Serial.println();
    #endif
    if(service[i].len != 0)
      readByType(service[i].init_handle,service[i].end_handle,service[i].id,service[i].len);
    i++;
  }
}

void BLE112::read_all_characteristics(){
  uint8_t i = 0;
  while(i<c_index){
    if(is_readable(char_[i].permissions)){
      #ifdef BLE_DEBUG
      Serial.printf("reading uuid: %x %x \n",(uint8_t)(char_[i].uuid>>8),(uint8_t)char_[i].uuid);
      #endif
      readByHandle(char_[i].handle);
    }
    i++;
  }
}

bool BLE112::char_handle_exists(uint16_t handle){
  uint8_t i = 0;
  while(i<c_index){
    if(char_[i].handle == handle)
      return true;
    i++;
  }
  return false;
}


bool BLE112::char_uuid_exists(uint16_t uuid){
  uint8_t i = 0;
  while(i<c_index){
    if(char_[i].uuid == uuid){
      #ifdef BLE_DEBUG
      Serial.printf("uuid exists: %d \n",uuid);
      #endif
      return true;
    }
    i++;
  }
  return false;
}

uint16_t BLE112::get_permission(uint16_t uuid){
  uint8_t i = 0;
  while(i<c_index){
    if(char_[i].uuid == uuid){
      is_readable(char_[i].permissions);
      is_write_no_response(char_[i].permissions);
      is_writable(char_[i].permissions);
      has_notify(char_[i].permissions);
      has_indicate(char_[i].permissions);
      has_reliable_write(char_[i].permissions);
      return char_[i].permissions;
    }
    i++;
  }
  return 0;
}

uint16_t BLE112::get_handle(uint16_t uuid){
  uint8_t i = 0;
  while(i<c_index){
    if(char_[i].uuid == uuid){
      return char_[i].handle;
    }
    i++;
  }
  return 0;
}


bool BLE112::getSystemInfo(){

  msg.length = HEADER_SIZE;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_SYSTEM;
  msg.cmd = 0x08;

  return sendCommand(WAIT_RESPONSE,1000);
}


bool BLE112::indicate(uint16_t uuid){
  if(char_uuid_exists(uuid)){
    uint16_t handle = get_handle(uuid);
    uint8_t data[] = {0x02, 0x00}; // indicate
    if(writeCommand(handle+1,data,2)){
      delay(200);
      checkMessages();
      return true;
    }
  }
  return false;
}


bool BLE112::indicateConfirm(){

  ble->flush();
  readCommand(0); // Empty rx buffer

  command[0] = 0; // message type
  command[1] = 1; // payload
  command[2] = 4; // Message class: Attribute Client
  command[3] = 7; // Message ID
  command[4] = connection;

  #ifdef BLE_DEBUG
  Serial.print("indicate confirm command: ");
  PrintHex8(command,5);
  #endif
  ble->write(command, 5);
  ble->flush();
}


bool BLE112::notify (uint16_t uuid){
  if(char_uuid_exists(uuid)){
    uint16_t handle = get_handle(uuid);
    uint8_t data[] = {0x01, 0x00}; // notify
    Serial.printf("handle: %d \n",handle+1);
    return writeCommand(handle+1,data,2);
  }else{
    Serial.printf("uuid: %x not found\n",uuid);
  }
  return false;
}

bool BLE112::notify_handle (uint16_t handle){

  uint8_t i = 0;
  while(i<c_index){
    if(char_[i].handle == handle){
      Serial.println("handle found");
      if(!has_notify(char_[i].permissions))
        return false;
      else break;
    }
    i++;
  }

  uint8_t data[] = {0x01, 0x00}; // notify
  return writeCommand(handle+1,data,2);
}

// use it write on an attribute
bool BLE112::write_attribute(uint16_t uuid,uint8_t* data, uint8_t len){
  if(char_uuid_exists(uuid)){
    uint16_t handle = get_handle(uuid);
    return attributeWrite(handle,data,len);
  }
  return false;
}

// use it for indicate and notify
bool BLE112::write_command(uint16_t uuid,uint8_t* data, uint8_t len){
  if(char_uuid_exists(uuid)){
    uint16_t permission = get_permission(uuid);
    if(is_writable(permission) || is_write_no_response(permission) || has_reliable_write(permission)){
      uint16_t handle = get_handle(uuid);
      return writeCommand(handle,data,len);
    }
  }
  return false;
}

bool BLE112::read_uuid(uint16_t uuid){
  if(char_uuid_exists(uuid)){
    uint16_t handle = get_handle(uuid);
    return readByHandle(handle);
  }
  return false;
}


// --- Communicate with external BLE devices ---
/*
  Attribute Write
  This command can be used to write an attributes value on a remote device. In order to write the value of an
  attribute a connection must exists and you need to know the handle of the attribute you want to write. Bluetooth
  A successful attribute write will be acknowledged by the remote device and this will generate an event
  attclient_procedure_completed. The acknowledgement should happen within a 30 second window or otherwise
  the Bluetooth connection will be dropped.
  The data payload for the Attribute Write command can be up to 20 bytes.
*/
bool BLE112::attributeWrite(uint16_t atthandle,uint8_t* data, uint8_t size){
  uint8_t len = HEADER_SIZE+4+size;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = len-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_ATTRIBUTE_WRITE;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)atthandle;
  msg.data[2] = (uint8_t)(atthandle>>8);
  msg.data[3] = size;

  i = 0;
  while(i<size){
    msg.data[4+i] = data[i];
    i++;
  }


  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
  return false;
}

/*
  Execute Write
  This command can be used to execute or cancel a previously queued prepare_write command on a remote
  device.
*/
bool BLE112::executeWrite(uint8_t commit){

  msg.length = HEADER_SIZE+2;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_EXECUTE_WRITE;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)commit;

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
}

/*
  Find By Type Value

  This command can be used to find specific attributes on a remote device based on their 16-bit UUID value and
  value. The search can be limited by a starting and ending handle values.
  The command returns the handles of all attributes matching the type (UUID) and value
*/
bool BLE112::findByTypeValue(uint16_t uuid,uint16_t start, uint16_t end,uint8_t* value, uint8_t size){
  uint8_t len = HEADER_SIZE+8+size;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length - HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_REMOTE_FIND_BY_TYPE_VALUE;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)start;
  msg.data[2] = (uint8_t)(start>>8);
  msg.data[3] = (uint8_t)end;
  msg.data[4] = (uint8_t)(end>>8);
  msg.data[5] = (uint8_t)uuid;
  msg.data[6] = (uint8_t)(uuid>>8);
  msg.data[7] = (uint8_t)size;

  i = 0;
  while(i<size){
    msg.data[8+i] = value[i];
    i++;
  }

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
  return false;
}

/*
  Find Information
  This command is used to discover attribute handles and their types (UUIDs) in a given handle range
*/
bool BLE112::findInformation(uint16_t start, uint16_t end){
  uint8_t len = HEADER_SIZE+5;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length - HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_READ_REMOTE_FIND_INFORMATION;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)start;
  msg.data[2] = (uint8_t)(start>>8);
  msg.data[3] = (uint8_t)end;
  msg.data[4] = (uint8_t)(end>>8);

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
}

/*
Indicate Confirm
This command can be used to send a acknowledge a received indication from a remote device. This function
allows the application to manually confirm the indicated values instead of the smart stack Bluetooth
automatically doing it. The benefit of this is extra reliability since the application can for example store the
received value on the flash memory before confirming the indication to the remote device.
*/
/*
bool BLE112::indicateConfirm(){

  msg.length = HEADER_SIZE+1;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_INDICATE_CONFIRM;
  msg.data[0] = connection;

  return sendCommand(WAIT_RESPONSE,1000);
}
*/

/*
  Prepare Write
  This command will send a prepare write request to a remote device for queued writes. Queued writes can for
  example be used to write large attribute values by transmitting the data in chunks using prepare write
  command.
  Once the data has been transmitted with multiple prepare write commands the write must then be executed or
  canceled with Execute Write command, which if acknowledged by the remote device triggers a Procedure
  Completed event.
*/
bool BLE112::prepareWrite(uint16_t atthandle,uint16_t offset,uint8_t* data, uint8_t size){
  uint8_t len = HEADER_SIZE+6+size;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = len-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_PREPARE_WRITE;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)atthandle;
  msg.data[2] = (uint8_t)(atthandle>>8);
  msg.data[1] = (uint8_t)offset;
  msg.data[2] = (uint8_t)(offset>>8);
  msg.data[3] = size;

  i = 0;
  while(i<size){
    msg.data[4+i] = data[i];
    i++;
  }

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
  return false;
}


/*
  Read By Group Type
  This command reads the value of each attribute of a given type and in a given handle range.
  The command is typically used for primary (UUID: 0x2800) and secondary (UUID: 0x2801) service discovery.
  Discovered services are reported by Group Found event.
  Finally when the procedure is completed a Procedure Completed event is generated.
*/
bool BLE112::readByGroupType(uint16_t start, uint16_t end,uint8_t* uuid, uint8_t size){
  uint8_t len = HEADER_SIZE+6+size;
  if(len>MAX_PACKET_SIZE)
    return false;

  s_index = 0;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length - HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_REMOTE_READ_BY_GROUP_TYPE;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)start;
  msg.data[2] = (uint8_t)(start>>8);
  msg.data[3] = (uint8_t)end;
  msg.data[4] = (uint8_t)(end>>8);
  msg.data[5] = (uint8_t)size;

  i = 0;
  while(i<size){
    msg.data[6+i] = uuid[i];
    i++;
  }

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
  return false;
}

/*
  Read By Handle
  This command reads a remote attribute's value with the given handle. Read by handle can be used to read
  attributes up to 22 bytes long.
  For longer attributes command must be used.
*/
bool BLE112::readByHandle(uint16_t atthandle){
  uint8_t len = HEADER_SIZE+3;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_READ_REMOTE_BY_HANDLE;


  msg.data[0] = connection;
  msg.data[1] = (uint8_t)atthandle;
  msg.data[2] = (uint8_t)(atthandle>>8);

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,1000); // in case of acknowledge by remote device
  }
}

/*
  Read By Type
  The command reads the value of each attribute of a given type (UUID) and in a given attribute handle range.
  The command can for example be used to discover the characteristic declarations (UUID: 0x2803) within a
  service.
*/
bool BLE112::readByType(uint16_t start, uint16_t end,uint8_t* uuid, uint8_t size){
  uint8_t len = HEADER_SIZE+6+size;
  if(len>MAX_PACKET_SIZE)
    return false;

  c_index = 0;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length - HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_READ_REMOTE_BY_TYPE;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)start;
  msg.data[2] = (uint8_t)(start>>8);
  msg.data[3] = (uint8_t)end;
  msg.data[4] = (uint8_t)(end>>8);
  msg.data[5] = (uint8_t)size;

  i = 0;
  while(i<size){
    msg.data[6+i] = uuid[size-i-1];
    i++;
  }

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED,3000); // in case of acknowledge by remote device
  }
  return false;
}

/*
  Read Long

  This command can be used to read long attribute values, which are longer than 22 bytes and cannot be read
  with a simple Read by Handle command.
  The command starts a procedure, where the client first sends a normal read command to the server and if the
  returned attribute value length is equal to MTU, the client will send further read long read requests until rest of
  the attribute is read.
*/
bool BLE112::readLong(){
  return true;
}

/*
  Read Multiple

  This command can be used to read multiple attributes from a server
*/
bool BLE112::readMultiple(){
  return true;
}

/*
  Write Command
  Writes the value of a remote devices attribute. The handle and the new value of the attribute are gives as
  parameters.
*/
bool BLE112::writeCommand(uint16_t atthandle,uint8_t* data, uint8_t size){
  uint8_t len = HEADER_SIZE+4+size;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = len-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_ATTCLIENT;
  msg.cmd = BLE_REMOTE_WRITE_COMMAND;

  msg.data[0] = connection;
  msg.data[1] = (uint8_t)atthandle;
  msg.data[2] = (uint8_t)(atthandle>>8);
  msg.data[3] = size;

  i = 0;
  while(i<size){
    msg.data[4+i] = data[i];
    i++;
  }


  return sendCommand(WAIT_RESPONSE,1000);

}

// --- --- ---

// --- Connection methods ---

/*
  Disconnect
  This command disconnects an active connection. Bluetooth
  When link is disconnected a Disconnected event is produced.
*/
bool BLE112::connectionClose(){
  msg.length = HEADER_SIZE+1;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_CONNECTION;
  msg.cmd = BLE_REMOTE_DISCONNECT;
  msg.data[0] = connection;

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_DISCONNECT,1000);
  }else return false;
}

/*
  Get Rssi
  This command returns the Receiver Signal Strength Indication (RSSI) related to the connection referred to by
  the connection handle parameter. If the connection is not open, then the RSSI value returned in the response
  packet will be 0x00, while if the connection is active, then it will be some negative value (2's complement form
  between 0x80 and 0xFF and never 0x00). Note that this command also returns an RSSI of 0x7F if you request
  RSSI on an invalid/unsupported handle.
*/
bool BLE112::getRSSI(int8_t* rssi){
  msg.length = HEADER_SIZE;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_CONNECTION;
  msg.cmd = BLE_REMOTE_RSSI;

  if(sendCommand(WAIT_RESPONSE,1000)){
    *rssi = response[5];
    return true;
  }else return false;
}

/*
  Get Status
  This command returns the status of the given connection.
  Status is returned in a event.
*/
bool BLE112::getStatus(uint8_t* status){
  msg.length = HEADER_SIZE;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_CONNECTION;
  msg.cmd = BLE_REMOTE_CONNECTION_STATUS;

  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_CONNECTION_STATUS,1000);
  }else return false;
}

/* NOT Implemented
  Slave Latency Disable
  This command temporarily enables or disables slave latency
*/
bool BLE112::slaveLatencyDisable(){
  return true;
}

/* NOT Implemented
  Update
  This command updates the connection parameters of a given connection. The parameters have the same
  meaning and follow the same rules as for the GAP class command: Connect Direct.
  If this command is issued at a master device, it will send parameter update request to the link layer. Bluetooth
  On the other hand if this command is issued at a slave device, it will send L2CAP connection parameter update
  request to the master, which may either accept or reject it.
  Silicon Labs Page of 90 227
  It will take an amount of time corresponding to at least six times the current connection interval before the new
  connection parameters will become active.
*/
bool BLE112::update(){


  return true;
}


/* NOT Implemented
  Version Update
  This command requests a version exchange of a given connection.
*/
bool BLE112::versionUpdate(){

  // wait event

  return true;
}

// --- GAP methods ---
/*
  Connect Direct
  This command will start the GAP direct connection establishment procedure to a dedicated Smart Bluetooth
  device.
  The module will enter a state where it continuously scans for the connectable advertisement packets Bluetooth
  from the remote device which matches the Bluetooth address gives as a parameter. Upon receiving the
  advertisement packet, the module will send a connection request packet to the target device to imitate a
  Bluetooth connection. A successful connection will bi indicated by a event. Status
  If the device is configured to support more than one connection, the smallest connection interval which is
  divisible by maximum_connections * 2.5ms will be selected. Thus, it is important to provide minimum and
  maximum connection intervals so that such a connection interval is available within the range.
  The connection establishment procedure can be cancelled with End Procedure command.

  0: gap_address_type_public
  1: gap_address_type_random
*/
bool BLE112::connectDirect(uint8_t* address){
  uint8_t addr_type = scan.address_type;
  uint16_t conn_interval_min = 32;
  uint16_t conn_interval_max = 48;
  uint16_t timeout = 100;
  uint16_t latency = 0;
  // wait event

  uint8_t len = HEADER_SIZE+15;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = len-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_REMOTE_CONNECT;

  i = 0;
  while(i<6){
    msg.data[i] = address[5-i];
    i++;
  }
  msg.data[i++] = addr_type;
  msg.data[i++] = conn_interval_min;
  msg.data[i++] = (uint8_t)(conn_interval_min>>8);
  msg.data[i++] = conn_interval_max;
  msg.data[i++] = (uint8_t)(conn_interval_max>>8);
  msg.data[i++] = timeout;
  msg.data[i++] = (uint8_t)(timeout>>8);
  msg.data[i++] = latency;
  msg.data[i++] = (uint8_t)(latency>>8);

  Serial.println("\nconnect to: ");
  PrintHex8_inverted(&msg.data[0],6);
  if(sendCommand(WAIT_RESPONSE,1000)){
    //return waitEvent(BLE_CLASS_ID_CONNECTION,BLE_EVENT_CONNECTION_STATUS,3000); // in case of acknowledge by remote device
    return waitEvent(BLE_EVENT_CONNECTION_STATUS,3000); // in case of acknowledge by remote device
  }
  return false;
}

bool BLE112::connectSelective(){
  uint16_t conn_interval_min = 60;
  uint16_t conn_interval_max = 76;
  uint16_t timeout = 100;
  uint16_t latency = 0;
  // wait event

  uint8_t len = HEADER_SIZE+9;
  if(len>MAX_PACKET_SIZE)
    return false;

  msg.length = len;
  msg.mt = BLE_MESSAGE;
  msg.ll = len-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_REMOTE_CONNECT_SELECTIVE;

  msg.data[i++] = conn_interval_min;
  msg.data[i++] = (uint8_t)(conn_interval_min>>8);
  msg.data[i++] = conn_interval_max;
  msg.data[i++] = (uint8_t)(conn_interval_max>>8);
  msg.data[i++] = timeout;
  msg.data[i++] = (uint8_t)(timeout>>8);
  msg.data[i++] = latency;
  msg.data[i++] = (uint8_t)(latency>>8);


  if(sendCommand(WAIT_RESPONSE,1000)){
    return waitEvent(BLE_EVENT_CONNECTION_STATUS,1000); // in case of acknowledge by remote device
  }
  return false;
}


// --- --- ---
/*
  Discover
  This command starts the GAP discovery procedure to scan for advertising devices i.e. to perform a device
  discovery.
  Scanning parameters can be configured with the Set Scan Parameters command before issuing this command.
  To cancel on an ongoing discovery process use the End Procedure command.

  mode:
    - 0: gap_discover_limited
    - 1: gap_discover_generic
    - 2: gap_discover_observation
*/
bool BLE112::discover(uint8_t mode){
  msg.length = HEADER_SIZE+1;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_DISCOVER;
  msg.data[0] = mode;

  return sendCommand(WAIT_RESPONSE,1000);
}

/*
  End Procedure
  This command ends the current GAP discovery procedure and stop the scanning of advertising devices.
*/
bool BLE112::endProcedure(uint8_t mode){
  msg.length = HEADER_SIZE;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_END_PROCEDURE;

  return sendCommand(WAIT_RESPONSE,1000);
}

/* NOT Implemented
  Set Adv Data
  This commands set advertisement or scan response data used in the advertisement and scan response
  packets. The command allows application specific data to be broadcasts either in advertisement or scan
  response packets.
  The data set with this command is only used when the GAP discoverable mode is set to gap_user_data.
  Notice that advertisement or scan response data must be formatted in accordance to the Bluetooth Core
  Specification. See BLUETOOTH SPECIFICATION Version 4.0 [Vol 3 - Part C - Chapter 11].
*/
bool BLE112::setAdvData(){
  return false;
}

/* NOT Implemented
  Set Directed Connectable Mode
  This command sets device to Directed Connectable mode.
  In this mode the device uses fast advertisement procedure for the first 1.28 seconds, after which the device
  enters a non-connectable mode. If the device implements the Peripheral Preferred Connection Parameters
  characteristic in its GAP service the parameters defined by this characteristic will be used for the connection.
*/
bool BLE112::setDirConnectableMode(){
  return false;
}

/*
  Set Initiating Con Parameters
  This command sets the scan parameters for Initiating State which affect for establishing BLE connection
*/
bool BLE112::setInitiatingConParameters(){
  return false;
}

/*
  Set Mode
  This command configures the current GAP discoverability and connectability modes. It can be used to enable
  advertisements and/or allow connection. The command is also meant to fully stop advertising, when using
  gap_non_discoverable and gap_non_connectable.

  GAP DISCOVERABLE MODE
  0 - gap_non_discoverable
  1 - gap_limited_discoverable
  2 - gap_general_discoverable
  3 - gap_broadcast
  4 - gap_user_data
  0x80 - gap_enhanced_broadcasting

  GAP Connectable Mode
  0 - gap_non_connectable
  1 - gap_directed_connectable
  2 - gap_undirected_connectable
  3 - gap_scannable_non_connectable
*/
bool BLE112::setMode(uint8_t discover, uint8_t connect){
  msg.length = HEADER_SIZE+2;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_DISCOVER;
  msg.data[0] = discover;
  msg.data[1] = connect;

  return sendCommand(WAIT_RESPONSE,1000);
}



/*
  Set Privacy Flags
  This command sets GAP central/peripheral privacy flags.
  By setting for example peripheral_privacy to 1, the stack will automatically generate a resolvable Bluetooth
  random private address for the advertising packets every time the command is used to enter Set Mode
  advertising mode.
  By setting privacy mode to 2, the stack will generate a resolvable random private address on Bluetooth
  demand. If peripherial_privacy is set to 2 additionally is called with the current Discoverable and Set Mode
  Connectable parameters. Setting up new mode by command does not change generated address

  peripheral_privacy
  2: change peripheral private address on demand
  1: enable peripheral privacy
  0: disable peripheral privacy
  Any other value will have no effect on flag

  central_privacy
  2: change central private address on demand
  1: enable central privacy
  0: disable central privacy
  Any other value will have no effect on flag
*/
bool BLE112::setPrivacyFlags(uint8_t peripheral_privacy, uint8_t central_privacy){
  msg.length = HEADER_SIZE+2;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_DISCOVER;
  msg.data[0] = peripheral_privacy;
  msg.data[1] = central_privacy;

  return sendCommand(WAIT_RESPONSE,1000);
}

/*
Set Scan Parameters
This command sets the scan parameters which affect how other Smart devices are discovered.
*/
bool BLE112::setScanParameters(uint16_t scan_interval, uint16_t scan_window, uint8_t active){
  msg.length = HEADER_SIZE+5;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_GAP;
  msg.cmd = BLE_SET_SCAN_PARAMETERS;
  msg.data[0] = scan_interval;
  msg.data[1] = (uint8_t)(scan_interval>>8);
  msg.data[2] = scan_window;
  msg.data[3] = (uint8_t)(scan_window>>8);
  msg.data[4] = active;

  return sendCommand(WAIT_RESPONSE,1000);
}

/* NOT Implemented
  Set Filtering
  This command can be used to set scan, connection, and advertising filtering parameters based on the local
  devices white list. See also Whitelist Append command.
*/
bool BLE112::setFiltering(){
  return true;
}

/*
  Set RXpower
  This command sets the radio receiver (RX) sensitivity to either high (default) or standard. The exact sensitivity
  value is dependent on the used hardware (refer to the appropriate data sheet).

  // gain
  0: standard gain
  1: high gain (default)
*/
bool BLE112::setRXgain(uint8_t power){
  msg.length = HEADER_SIZE+1;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_HARDWARE;
  msg.cmd = BLE_SET_RXGAIN;
  msg.data[0] = power;

  return sendCommand(WAIT_RESPONSE,1000);
}

/*
  Set TXpower
  Re-configure TX output power

  0 to 15 with the BLE112 and the BLED112
*/
bool BLE112::setPower(uint8_t power){
  msg.length = HEADER_SIZE+1;
  msg.mt = BLE_MESSAGE;
  msg.ll = msg.length-HEADER_SIZE;
  msg.cid = BLE_CLASS_ID_HARDWARE;
  msg.cmd = BLE_SET_POWER;
  msg.data[0] = power;

  return sendCommand(WAIT_RESPONSE,1000);
}

// --- private ---

bool BLE112::is_readable(uint8_t permission){
  #ifdef BLE_DEBUG
    Serial.println("is_readable");
  #endif
  return (permission & 0x02);
}

bool BLE112::is_write_no_response(uint8_t permission){
  #ifdef BLE_DEBUG
    Serial.println("is_write_no_response");
  #endif
  return (permission & 0x04) > 0;
}

bool BLE112::is_writable(uint8_t permission){
  #ifdef BLE_DEBUG
    Serial.println("is_writable");
  #endif
  return (permission & 0x08) > 0;
}

bool BLE112::has_notify(uint8_t permission){
  #ifdef BLE_DEBUG
    Serial.println("has_notify");
  #endif
  return (permission & 0x10) > 0;
}

bool BLE112::has_indicate(uint8_t permission){
  #ifdef BLE_DEBUG
    Serial.println("has_indicate");
  #endif
  return (permission & 0x20) > 0;
}

bool BLE112::has_reliable_write(uint8_t permission){
  #ifdef BLE_DEBUG
    Serial.println("has_reliable_write");
  #endif
  return (permission & 0x80) > 0;
}

bool BLE112::waitEvent(uint8_t class_,uint8_t event,uint16_t timeout){
  return false;
}

bool BLE112::waitEvent(uint8_t event,uint16_t timeout){
  /*
  #ifdef BLE_DEBUG
    Serial.print("data rcv: ");
    PrintHex8(response,inc);
  #endif
  */
  uint32_t time = millis() + timeout;
  while(millis() < time){
    if(ble->available()){
      readCommand(0);
      if(response[0] == 0x80 && response[3] == event)
        return true;
    }
  }
  return false;
}


bool BLE112::parseCommand(){
  if(msg.mt == response[0] && msg.cid == response[2] && msg.cmd == response[3]){
    if(response[4] == 0 && response[5] == 0){
      #ifdef BLE_DEBUG
        Serial.println("Command sent successful");
      #endif
      return true;
    }else if(response[4] != 0 || response[5] != 0){
      #ifdef BLE_DEBUG
        Serial.printf("Error sending command \n");
      #endif
      parseError((uint16_t)(response[4]<<8)|response[5]);
      return false;
    }else{
      #ifdef BLE_DEBUG
        Serial.println("msg response does not correspond to the message sent");
      #endif
      return false;
    }
  }
}


bool BLE112::parseEvent(uint8_t len){

  if( response[2] == BLE_CLASS_ID_SYSTEM && response[3] == 0x06){
    Serial.println("An error ocurred");
    parseError((uint16_t)(response[4]|(response[5]>>8)));
  }else if(response[2] == BLE_CLASS_ID_SYSTEM && response[3] == 0x00){
    Serial.println("--- ble has booted ---");
    module.major_sw_version = response[4]|(response[5]>>8);
    module.minor_sw_version = response[6]|(response[7]>>8);
    module.patch = response[8]|(response[9]>>8);
    module.build = response[10]|(response[11]>>8);
    module.ll_version = response[12]|(response[13]>>8);
    module.protocol_version = response[14];
    module.hw_version = response[15];
    #ifdef BLE_DEBUG
      Serial.printf("version: %d.%d \n",module.major_sw_version,module.minor_sw_version);
      Serial.printf("link layer version: %d \n",module.ll_version);
      Serial.printf("protocol version: %d \n",module.protocol_version);
      Serial.printf("hw version: %d \n",module.hw_version);
      Serial.println("--- --- ---");
    #endif
  }else if(response[2] == BLE_CLASS_ID_CONNECTION){

    if(response[3] == BLE_EVENT_CONNECTION_STATUS){
      Serial.println("--- Connection status ---");
      slave.connected = true;
      memcpy(slave.mac,scan.discover,6);
      switch(response[5]){
        case 0x00:
          Serial.println("Connection connected");
          break;
        case 0x01:
          Serial.println("Connection encrypted");
          break;
        case 0x02:
          Serial.println("Connection completed");
          break;
        case 0x04:
          Serial.println("Connection parameters change");
          break;
      }
    }else if(response[3] == BLE_EVENT_REMOTE_DEVICE_VERSION){
      Serial.println("--- Remote ---");
      Serial.printf("version: %d \n",response[5]);
      Serial.println("--- --- ---");
    }else if(response[3] == BLE_EVENT_DISCONNECT){
      Serial.println("Remote device has disconnected");
      slave.connected = false;
      uint16_t error = (uint16_t)(response[5]|(response[6]<<8));
      parseError(error);
    }

  }else if(response[2] == BLE_CLASS_ID_ATTCLIENT){
    if(response[3] == BLE_REMOTE_READ_BY_GROUP_TYPE){
      if(response[5] != 0 || response[6] != 0){
        Serial.println("!! Procedure BLE_REMOTE_READ_BY_GROUP_TYPE completed with error");
        uint16_t error = (uint16_t)(response[5]|(response[6]<<8));
        parseError(error);
      }else Serial.println("Procedure BLE_REMOTE_READ_BY_GROUP_TYPE completed");

    }else if (response[3] == BLE_EVENT_ATTCLIENT_GROUP_FOUND){
      if(s_index == MAX_SERVICES){
        Serial.println("Service array is full");
        return false;
      }
      Serial.println("att client group found");
      service[s_index].init_handle = (uint16_t)response[5]|(response[6]<<8);
      service[s_index].end_handle = (uint16_t)response[7]|(response[8]<<8);
      service[s_index].len = response[9];
      Serial.println();
      uint8_t i = 0;
      while(i<response[9] && i<SERVICE_SIZE){
        service[s_index].id[i] = response[10+i]; // first byte is the len
        i++;
      }
      #ifdef BLE_DEBUG
        Serial.printf("init handle: %d \n",service[s_index].init_handle);
        Serial.printf("end handle: %d \n",service[s_index].end_handle);
        Serial.print("Service: ");
        PrintHex8(&service[s_index].id[0],service[s_index].len);
        Serial.println();
      #endif
      if(s_index < MAX_SERVICES)
        s_index++;

    }else if(response[3] == BLE_EVENT_ATTCLIENT_ATTRIBUTE_VALUE){

      //#ifdef BLE_HIGH_DEBUG
      uint16_t handle = (uint16_t)response[5]|(response[6]<<8);

      #ifdef BLE_HIGH_DEBUG
        Serial.printf("type: %d \n",(uint8_t)response[7]);
      #endif

      #ifdef BLE_HIGH_DEBUG
      Serial.print("full msg: ");
      PrintHex8(&response[0],response_size);
      #endif

      #ifdef BLE_HIGH_DEBUG
      Serial.print("data rcv: ");
      PrintHex8(&response[9],response[8]);
      #endif

      parse_rcv_data(handle,&response[9],response[8]);

      //#endif
      memset(returned_data,0,MAX_DATA);
      memcpy(returned_data,&response[9],response[8]);

      if(c_index == MAX_CHARACTERISTICS){
        Serial.println("Characteristics array is full");
        return false;
      }
      char_[c_index].type = response[7];
      if(response[8] >= 5){
        char_[c_index].permissions = response[9];
        char_[c_index].handle = (uint16_t)response[10]|(response[11]<<8);
        char_[c_index].uuid = (uint16_t)response[12]|(response[13]<<8);
        if(!char_handle_exists(char_[c_index].handle))
          c_index++;
      }

    }else if(response[3] == BLE_ATTRIBUTE_WRITE){
      #ifdef BLE_DEBUG
        Serial.println("att client attribute write");
        Serial.printf("attribute handle: %d \n",(uint16_t)response[5]|(response[6]<<8));
        Serial.printf("type: %d",response[7]);
        Serial.print("value received: ");
        PrintHex8(&response[9],response[8]);
        Serial.println();
      #endif

    }else{
      PrintHex8(response,response[1]+2);
    }
  }else if(response[2] == BLE_CLASS_ID_GAP && response[3] == BLE_EVENT_GAP_SCAN_RESPONSE){
    //Serial.println("Scan event");
    uint8_t i = 0;
    while(i<6){
      scan.mac[i] = response[12-i-1];
      i++;
    }

    #ifdef BLE_HIGH_DEBUG
      switch(response[5]){
        case 0:
          Serial.println("Connectable adversisement packet");
          break;
        case 2:
          Serial.println("Non Connectable adversisement packet");
          break;
        case 4:
          Serial.println("Scan response packet");
          break;
        case 6:
          Serial.println("Discoverable adversisement packet");
          break;
        default: break;
      }

      PrintHex8_inverted(&response[6],6);
      Serial.print("mac: ");
      PrintHex8(scan.mac,6);

      if(response[12] == 1)
        Serial.println("address_type: random address");
      else
        Serial.println("address_type: public address");
      Serial.printf("bond: %d \n",response[13]);

      PrintHex8(&response[15],response[14]);
    #elif defined BLE_DEBUG
      //Serial.print("mac: ");
      //PrintHex8(scan.mac,6);
    #endif

    scan.rssi = response[4];
    scan.address_type = response[12];
    if ( memcmp( (const void *)scan.discover, (const void *)scan.mac, 6) == 0){
      scan.mac_found = true;
      slave.rssi = response[4];
      #ifdef BLE_HIGH_DEBUG
        Serial.print("adv packet data: ");
        PrintHex8(&response[15],response[14]);
      #endif
    }

  }else{
    Serial.println("!! EVENT NOT PARSED");
    PrintHex8(&response[0],len);
    return false;
  }

  return true;
}

// this function is always used to print debug messages.
void BLE112::PrintHex8(uint8_t *data, uint8_t length){ // prints 8-bit data in hex with leading zeroes

  for (uint8_t a = 0; a < length; a++)
  {
    if (data[a] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(data[a], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// this function is always used to print debug messages.
void BLE112::PrintHex8_inverted(uint8_t *data, uint8_t length){ // prints 8-bit data in hex with leading zeroes

  for (int16_t a = length-1; a >= 0; a--)
  {
    if (data[a] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(data[a], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// sendCommand
bool BLE112::sendCommand(bool waitResponse, uint32_t wait){

  ble->flush();
  readCommand(0); // Empty rx buffer

  uint8_t i = 0;
  command[0] = msg.length;
  command[1] = msg.mt;
  command[2] = msg.ll;
  command[3] = msg.cid;
  command[4] = msg.cmd;
  while(i<msg.ll){
    command[5+i] = msg.data[i];
    i++;
  }
  #ifdef BLE_DEBUG
    Serial.print("write command: ");
    PrintHex8(&command[0],command[0]+1);
  #endif
  ble->write(command, command[0]+1);
  ble->flush();

  delay(100);
  if(waitResponse)
    return readCommand(wait);
  return true;
}

// readCommand
bool BLE112::readCommand(uint32_t wait){
  memset(response,0,MAX_PACKET_SIZE+HEADER_SIZE);
  inc = 0;
  uint32_t timeout = millis() + wait;
  do{
    response_size = 0;
    if(ble->available()){
      response_size = 0;
      response[response_size++] = ble->read();
      uint16_t size = ble->peek()+4;
      if(size < (MAX_PACKET_SIZE+HEADER_SIZE)){
        while(response_size < size && response_size < (MAX_PACKET_SIZE+HEADER_SIZE)){
          response[response_size++] = ble->read();
        }
        #ifdef BLE_HIGH_DEBUG
          Serial.print("command: ");
          PrintHex8(response,size);
        #endif
        if(response[0] == 0x80){
          if(!parseEvent(response_size)) return false;
          else return true;
        }else return parseCommand();
      }else{
        while(ble->available()) ble->read();
        return false;
      }
    }
  } while(timeout > millis());
  return false;
}

// parseError
void BLE112::parseError(uint16_t e){
  Serial.printf("error: %x -> ", e);
  switch(e){
    case InvalidParameter:
      Serial.println("InvalidParameter");
      break;
    case DeviceWrongState:
      Serial.println("DeviceWrongState");
      break;
    case OutOfMemory:
      Serial.println("OutOfMemory");
      break;
    case FeatureNotImplemented:
      Serial.println("FeatureNotImplemented");
      break;
    case CommandNotRecognize:
      Serial.println("CommandNotRecognize");
      break;
    case Timeout:
      Serial.println("Timeout");
      break;
    case NotConnected:
      Serial.println("NotConnected");
      break;
    case flow:
      Serial.println("flow");
      break;
    case UserAttribute:
      Serial.println("UserAttribute");
      break;
    case InvalidLicenseKey:
      Serial.println("InvalidLicenseKey");
      break;
    case CommandTooLon:
      Serial.println("CommandTooLon");
      break;
    case OutofBond:
      Serial.println("OutofBond");
      break;
    case AuthenticationFailure:
      Serial.println("AuthenticationFailure");
      break;
    case PinorKeyMissing:
      Serial.println("PinorKeyMissing");
      break;
    case MemoryCapacityExceeded:
      Serial.println("MemoryCapacityExceeded");
      break;
    case ConnectionTimeout:
      Serial.println("ConnectionTimeout");
      break;
    case ConnectionLimitExceeded:
      Serial.println("ConnectionLimitExceeded");
      break;
    case CommandDisallowed:
      Serial.println("CommandDisallowed");
      break;
    case InvalidCommandParameter:
      Serial.println("InvalidCommandParameter");
      break;
    case RemoteUserTerminatedConnection:
      Serial.println("RemoteUserTerminatedConnection");
      break;
    case ConnectionTerminatedbyLocalHost:
      Serial.println("ConnectionTerminatedbyLocalHost");
      break;
    case LLResponseTimeout:
      Serial.println("LLResponseTimeout");
      break;
    case LLInstantPassed:
      Serial.println("LLInstantPassed");
      break;
    case ControllerBus:
      Serial.println("ControllerBus");
      break;
    case UnacceptableConnectionInterval:
      Serial.println("UnacceptableConnectionInterval");
      break;
    case DirectedAdvertisingTimeout:
      Serial.println("DirectedAdvertisingTimeout");
      break;
    case MICFailure:
      Serial.println("MICFailure");
      break;
    case ConnectionFailedtobeEstablished:
      Serial.println("ConnectionFailedtobeEstablished");
      break;
    case PasskeyEntryFailed:
      Serial.println("PasskeyEntryFailed");
      break;
    case OOBDataisnotavailable:
      Serial.println("OOBDataisnotavailable");
      break;
    case AuthenticationRequirements:
      Serial.println("AuthenticationRequirements");
      break;
    case ConfirmValueFailed:
      Serial.println("ConfirmValueFailed");
      break;
    case PairingNotSupported:
      Serial.println("PairingNotSupported");
      break;
    case EncryptionKeySize:
      Serial.println("EncryptionKeySize");
      break;
    case CommandNotSupported:
      Serial.println("CommandNotSupported");
      break;
    case UnspecifiedReason:
      Serial.println("UnspecifiedReason");
      break;
    case RepeatedAttempt:
      Serial.println("RepeatedAttempt");
      break;
    case InvalidParameters:
      Serial.println("InvalidParameters");
      break;
    case InvalidHandle:
      Serial.println("InvalidHandle");
      break;
    case ReadNotPermitte:
      Serial.println("ReadNotPermitte");
      break;
    case WriteNotPermitted:
      Serial.println("WriteNotPermitted");
      break;
    case InvalidPD:
      Serial.println("InvalidPD");
      break;
    case InsufficientAuthentication:
      Serial.println("InsufficientAuthentication");
      break;
    case RequestNotSupported:
      Serial.println("RequestNotSupported");
      break;
    case InvalidOffset:
      Serial.println("InvalidOffset");
      break;
    case InsufficientAuthorization:
      Serial.println("InsufficientAuthorization");
      break;
    case PrepareQueueFull:
      Serial.println("PrepareQueueFull");
      break;
    case AttributeNotFound:
      Serial.println("AttributeNotFound");
      break;
    case AttributeNotLong:
      Serial.println("AttributeNotLong");
      break;
    case InsufficientEncryptionKeySize:
      Serial.println("InsufficientEncryptionKeySize");
      break;
    case InvalidAttributeValueLength:
      Serial.println("InvalidAttributeValueLength");
      break;
    case UnlikelyError:
      Serial.println("UnlikelyError");
      break;
    case InsufficientEncryption:
      Serial.println("InsufficientEncryption");
      break;
    case UnsupportedGroupType:
      Serial.println("UnsupportedGroupType");
      break;
    case InsufficientResources:
      Serial.println("InsufficientResources");
      break;
    case ApplicationErrorCodes:
      Serial.println("ApplicationErrorCodes");
      break;
    default:
      Serial.println("Invalid error");
      break;
  }
}

/*
 * Function: Converts a string to an array of bytes
 * For example: If the input array -> 23576173706D6F74655F50726F23
 * The output string is str -> #Waspmote_Pro#
 */
uint16_t BLE112::str2hex(char* str, uint8_t* array){

  // get length in bytes (half of ASCII characters)
	uint16_t length=strlen(str)/2;

  // Conversion from ASCII to HEX
  for(uint16_t j=0; j<length; j++){
    array[j] = str2hex(&str[j*2]);
  }

	return length;
}

/*
 * Function: Converts a string to an hex number
 *
 */
uint8_t BLE112::str2hex(uint8_t* str){
	int aux=0, aux2=0;

	if( (*str>='0') && (*str<='9') )
	{
		aux=*str++-'0';
	}
	else if( (*str>='A') && (*str<='F') )
	{
		aux=*str++-'A'+10;
	}
	if( (*str>='0') && (*str<='9') )
	{
		aux2=*str-'0';
	}
	else if( (*str>='A') && (*str<='F') )
	{
		aux2=*str-'A'+10;
	}
	return aux*16+aux2;
}

/*
 * Function: Converts a string to an hex number
 *
 */
uint8_t BLE112::str2hex(char* str){
	int aux=0, aux2=0;


	if( (*str>='0') && (*str<='9') )
	{
		aux=*str++-'0';
	}
	else if( (*str>='A') && (*str<='F') )
	{
		aux=*str++-'A'+10;
	}
	if( (*str>='0') && (*str<='9') )
	{
		aux2=*str-'0';
	}
	else if( (*str>='A') && (*str<='F') )
	{
		aux2=*str-'A'+10;
	}
	return aux*16+aux2;
}
