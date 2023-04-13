
#ifndef BLE112_h
#define BLE112_h

#include "Arduino.h"
#include <inttypes.h>
#include <stdint.h>

//#define BLE_HIGH_DEBUG
//#define BLE_DEBUG

#define MAX_PACKET_SIZE 60
#define HEADER_SIZE 4

#define NO_RESPONSE 0x00
#define WAIT_RESPONSE 0x01

#define BLE_MESSAGE 0x00
#define BLE_EVENT 0x80

// COMMANDS CLASS ID
#define BLE_CLASS_ID_SYSTEM 0x00
#define BLE_CLASS_ID_STORE 0x01
#define BLE_CLASS_ID_DATABASE 0x02  // only use it if your ble is a sensor
#define BLE_CLASS_ID_CONNECTION 0x03 // manage bluetooth connections
#define BLE_CLASS_ID_ATTCLIENT 0x04 // use it to communicate with external sensors
#define BLE_CLASS_ID_SECMANAGER 0x05
#define BLE_CLASS_ID_GAP 0x06
#define BLE_CLASS_ID_HARDWARE 0x07

// --- ATTRIBUTE CLIENT ---

  // COMMANDS ID
  #define BLE_REMOTE_FIND_BY_TYPE_VALUE 0x00
  #define BLE_REMOTE_READ_BY_GROUP_TYPE 0x01
  #define BLE_READ_REMOTE_BY_TYPE 0x02
  #define BLE_READ_REMOTE_FIND_INFORMATION 0x03
  #define BLE_READ_REMOTE_BY_HANDLE 0x04
  #define BLE_ATTRIBUTE_WRITE 0x05
  #define BLE_REMOTE_WRITE_COMMAND 0x06
  #define BLE_INDICATE_CONFIRM 0x07
  #define BLE_REMOTE_READ_LONG 0x08
  #define BLE_PREPARE_WRITE 0x09
  #define BLE_EXECUTE_WRITE 0x0A
  #define BLE_REMOTE_READ_MULTPILE 0x0B


  // EVENTS ID
  #define BLE_EVENT_ATTCLIENT_INDICATED 0x00
  #define BLE_EVENT_ATTCLIENT_PROCEDURE_COMPLETED 0x01
  #define BLE_EVENT_ATTCLIENT_GROUP_FOUND 0x02
  #define BLE_EVENT_ATTCLIENT_FIND_INFORMATION_FOUND 0x04
  #define BLE_EVENT_ATTCLIENT_ATTRIBUTE_VALUE 0x05
  #define BLE_EVENT_ATTCLIENT_READ_MULTIPLE_RESPONSE 0x06


  // ENUMERATIONS ID
  //- How to use it ??
  #define BLE_ATTVALUE_TYPE_READ 0x00 // Value was read
  #define BLE_ATTVALUE_TYPE_NOTIFY 0x01 // Value was notified
  #define BLE_ATTVALUE_TYPE_INDICATE 0x02 // Value was indicated
  #define BLE_ATTVALUE_TYPE_READ_BY_TYPE 0x03 // Value was read
  #define BLE_ATTVALUE_TYPE_READ_BLOB 0x04 // Value was part of a long attribute
  #define BLE_ATTVALUE_TYPE_INDICATE_RSP_REQ 0x05 // Value was indicated and the remote device is
  /*
  0 attclient_attribute_value_type_read Value was read
  1 attclient_attribute_value_type_notify Value was notified
  2 attclient_attribute_value_type_indicate Value was indicated
  3 attclient_attribute_value_type_read_by_type Value was read
  4 attclient_attribute_value_type_read_blob Value was part of a long attribute
  5 attclient_attribute_value_type_indicate_rsp_req Value was indicated and the remote device is
  waiting for a confirmation.
  Indicate Confirm command can be used to send a
  confirmation.
  */

// --- ATTRIBUTE DATABASE --- (Not used)

// --- CONNECTION ---

  // COMANDS ID
  #define BLE_REMOTE_DISCONNECT 0x00
  #define BLE_REMOTE_RSSI 0x01
  #define BLE_REMOTE_UPDATE 0x02
  #define BLE_REMOTE_VERSION_UPDATE 0x03
  #define BLE_REMOTE_CONNECTION_STATUS 0x07
  #define BLE_SLAVE_LATENCY_DISABLE 0x09

  // EVENTS ID
  #define BLE_EVENT_CONNECTION_STATUS 0x00
  #define BLE_EVENT_REMOTE_DEVICE_VERSION 0x01
  #define BLE_EVENT_REMOTE_FEATURES 0x02
  #define BLE_EVENT_DISCONNECT 0x04

  // ENUMERATIONS ID
  //- How to use it ??
  /*
  bit 0 connection_connected This status flag tells the connection exists to a remote device.
  bit 1 connection_encrypted This flag tells the connection is encrypted.
  bit 2 connection_completed Connection completed flag, which is used to tell a new connection
  has been created.
  bit 3 connection_parameters_change This flag tells that connection parameters have changed and. It is
  set when connection parameters have changed due to a link layer
  operation.
  */


// --- GAP ---

  //COMANDS ID
  #define BLE_SET_PRIVACY_FLAGS 0x00
  #define BLE_SET_MODE 0x01
  #define BLE_DISCOVER 0x02
  #define BLE_REMOTE_CONNECT 0x03
  #define BLE_END_PROCEDURE 0x04
  #define BLE_REMOTE_CONNECT_SELECTIVE 0x05
  #define BLE_SET_SCAN_PARAMETERS 0x07


  // EVENTS ID --- GAP ---
  #define BLE_EVENT_GAP_SCAN_RESPONSE 0x00
  // #define BLE_EVENT_CONNECTION_STATUS -> defined on CONNECTION CLASS

// --- Hardware ---

  //COMANDS ID
  #define BLE_SET_RXGAIN 0x13
  #define BLE_SET_POWER 0x0C


struct packet{
  uint8_t length; // command length (excluding itself)
  uint8_t mt; // message type | payload length high bits
  uint8_t ll; // payload length low bytes
  uint8_t cid; // class id
  uint8_t cmd; // command id
  uint8_t data[60]; // payload
};

struct Module{
  uint16_t major_sw_version;
  uint16_t minor_sw_version;
  uint16_t patch;
  uint16_t build;
  uint16_t ll_version;
  uint8_t  protocol_version;
  uint8_t  hw_version;
};

struct Scan{
  uint8_t discover[6];  // mac to discover
  uint8_t address_type;
  uint8_t mac[6];       // last mac found
  bool mac_found;       // if mac == discover
  int8_t rssi;          // rssi of last mac found
};

struct Slave{
  uint8_t mac[6];
  bool connected;
  int8_t rssi;
  int8_t battery;
};


#define MAX_SERVICES 10
#define SERVICE_SIZE 64
struct services{
  uint16_t init_handle;
  uint16_t end_handle;
  uint8_t len;
  uint8_t id[SERVICE_SIZE];
};

#define MAX_CHARACTERISTICS 50
struct characteristics{
  uint8_t type;
  uint8_t permissions;
  uint16_t handle; // point to property handle
  uint16_t uuid;  // first byte is the len
};

#define MAX_DESCRIPTORS 50
struct descriptors{
  uint8_t type;
  uint8_t permissions;
  uint16_t handle;
  uint16_t uuid;  // first byte is the len
};

#define MAX_DATA 255


class BLE112{
  public:
    BLE112(uint32_t baudrate,uint8_t gpio);
    bool initModule();
    void hardwareReset();
    uint8_t softwareReset();

    bool findMac(uint8_t* mac,uint32_t timeout);
    bool findMac(uint8_t* mac,uint32_t timeout, uint8_t mode);
    void checkMessages();
    bool hello();
    void list_characteristics();
    void get_all_characteristics();
    void read_all_characteristics();
    bool char_uuid_exists(uint16_t uuid);
    uint16_t get_handle(uint16_t uuid);
    uint16_t get_permission(uint16_t uuid);
    bool char_handle_exists(uint16_t handle);
    bool getSystemInfo();
    bool set_callback(void(*parseEvents)(uint16_t, uint8_t* data, uint8_t len)); // set callback to deal with rcv data
    void (*parse_rcv_data)(uint16_t, uint8_t* data, uint8_t len) = NULL; // function to deal with rcv data - ND

    bool indicate(uint16_t uuid);
    bool notify(uint16_t uuid);
    bool notify_handle(uint16_t uuid);
    bool write_attribute(uint16_t uuid,uint8_t* data, uint8_t len);
    bool write_command(uint16_t uuid,uint8_t* data, uint8_t len);
    bool read_uuid(uint16_t uuid);

    bool attributeWrite(uint16_t atthandle,uint8_t* data, uint8_t size);
    bool executeWrite(uint8_t commit);
    bool findByTypeValue(uint16_t uuid,uint16_t start, uint16_t end,uint8_t* value, uint8_t size);
    bool findInformation(uint16_t start, uint16_t end);
    bool indicateConfirm();
    bool prepareWrite(uint16_t atthandle,uint16_t offset,uint8_t* data, uint8_t size);
    bool readByGroupType(uint16_t start, uint16_t end,uint8_t* uuid, uint8_t size);
    bool readByHandle(uint16_t atthandle);
    bool readByType(uint16_t start, uint16_t end,uint8_t* uuid, uint8_t size);
    bool readLong();
    bool readMultiple();
    bool writeCommand(uint16_t atthandle,uint8_t* data, uint8_t size);
    bool connectionClose();
    bool getRSSI(int8_t* rssi);
    bool getStatus(uint8_t* status);
    bool slaveLatencyDisable();
    bool update();
    bool versionUpdate();
    bool connectDirect(uint8_t* address);
    bool connectSelective();
    bool discover(uint8_t mode);
    bool endProcedure(uint8_t mode);
    bool setAdvData();
    bool setDirConnectableMode();
    bool setInitiatingConParameters();
    bool setMode(uint8_t discover, uint8_t connect);
    bool setPrivacyFlags(uint8_t peripheral_privacy, uint8_t central_privacy);
    bool setScanParameters(uint16_t scan_interval, uint16_t scan_window, uint8_t active);
    bool setFiltering();
    bool setRXgain(uint8_t power);
    bool setPower(uint8_t power);

    bool waitEvent(uint8_t event,uint16_t timeout);
    bool waitEvent(uint8_t class_,uint8_t event,uint16_t timeout);

    void PrintHex8(uint8_t *data, uint8_t length);
    void PrintHex8_inverted(uint8_t *data, uint8_t length);

    uint16_t str2hex(char* str, uint8_t* array);
    uint8_t str2hex(uint8_t* str);
    uint8_t str2hex(char* str);

    uint8_t returned_data[MAX_DATA];
    struct Slave slave;

  private:

    bool is_readable(uint8_t permission);
    bool is_write_no_response(uint8_t permission);
    bool is_writable(uint8_t permission);
    bool has_notify(uint8_t permission);
    bool has_indicate(uint8_t permission);
    bool has_reliable_write(uint8_t permission);

    bool parseEvent(uint8_t len);
    bool parseCommand();
    bool sendCommand(bool waitReponse, uint32_t wait);
    bool readCommand(uint32_t wait);
    void checkEvents();
    void parseError(uint16_t error);

};


// Errors
//Errors related to BGAPI protocol
#define InvalidParameter 0x0180
#define DeviceWrongState 0x0181
#define OutOfMemory 0x0182
#define FeatureNotImplemented 0x0183
#define CommandNotRecognize 0x0184
#define Timeout 0x0185
#define NotConnected 0x0186
#define flow 0x0187
#define UserAttribute 0x0188
#define InvalidLicenseKey 0x0189
#define CommandTooLon 0x018A
#define OutofBond 0x018B

//Bluetooth Error
#define AuthenticationFailure 0x0205
#define PinorKeyMissing 0x0206
#define MemoryCapacityExceeded 0x0207
#define ConnectionTimeout 0x0208
#define ConnectionLimitExceeded 0x0209
#define CommandDisallowed 0x020C
#define InvalidCommandParameter 0x0212
#define RemoteUserTerminatedConnection 0x0213
#define ConnectionTerminatedbyLocalHost 0x0216
#define LLResponseTimeout 0x0222
#define LLInstantPassed 0x0228
#define ControllerBus 0x023A
#define UnacceptableConnectionInterval 0x023B
#define DirectedAdvertisingTimeout 0x023C
#define MICFailure 0x023D
#define ConnectionFailedtobeEstablished 0x023E

//SecurityManagerProtocolError
#define PasskeyEntryFailed 0x0301
#define OOBDataisnotavailable 0x0302
#define AuthenticationRequirements 0x0303
#define ConfirmValueFailed 0x0304
#define PairingNotSupported 0x0305
#define EncryptionKeySize 0x0306
#define CommandNotSupported 0x0307
#define UnspecifiedReason 0x0308
#define RepeatedAttempt 0x0309
#define InvalidParameters 0x030A


//AttributeProtocolError
#define InvalidHandle 0x0401
#define ReadNotPermitte 0x0402
#define WriteNotPermitted 0x0403
#define InvalidPD 0x0404
#define InsufficientAuthentication 0x0405
#define RequestNotSupported 0x0406
#define InvalidOffset 0x0407
#define InsufficientAuthorization 0x0408
#define PrepareQueueFull 0x0409
#define AttributeNotFound 0x040A
#define AttributeNotLong 0x040B
#define InsufficientEncryptionKeySize 0x040C
#define InvalidAttributeValueLength 0x040D
#define UnlikelyError 0x040E
#define InsufficientEncryption 0x040F
#define UnsupportedGroupType 0x0410
#define InsufficientResources 0x0411
#define ApplicationErrorCodes 0x0480

#endif
