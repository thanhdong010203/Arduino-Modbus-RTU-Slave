/*
#include "Modbus_RTU_Slave.h"
//setup
 Serial.begin(9600);
 Serial.setTimeout(10);
*/

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 64
#endif
#ifndef SLAVE_ID
#define SLAVE_ID 1
#endif
#define ILLEGAL_FUNCTION 0x01
#define ILLEGAL_DATA_ADDRESS 0x02
#define ILLEGAL_DATA_VALUE 0x03
//********************************
// khai bao bien
byte TxData[BUFFER_SIZE];
byte RxData[BUFFER_SIZE];
uint8_t RxLength = 0;
////////////////////////////////////////////////////////
static uint8_t Coils_0x[BUFFER_SIZE];
static uint8_t Inputs_1x[BUFFER_SIZE];
static uint16_t Holding_Registers_4x[BUFFER_SIZE];
static uint16_t Input_Registers_3x[BUFFER_SIZE];
//********************************/

////////////////////////////////////////////////
void   Write_bit_0x(uint16_t Address, bool Value);
bool      Read_bit_0x(uint16_t Address);
#define   Set_ON_0x(Address)   Write_bit_0x(Address, 1)
#define   Set_OFF_0x(Address)   Write_bit_0x(Address, 0)
void   Write_bit_1x(uint16_t Address, bool Value);
bool      Read_bit_1x(uint16_t Address);
#define   Set_ON_1x(Address)   Write_bit_1x(Address, 1)
#define   Set_OFF_1x(Address)   Write_bit_1x(Address, 0)
void      Write_4x(uint16_t Address, uint16_t Value);
uint16_t  Read_4x(uint16_t add);
void      WriteFloat_4x(uint16_t add, float value);
float     ReadFloat_4x(uint16_t add);
void      Write_3x(uint16_t add, uint16_t value);
uint16_t  Read_3x(uint16_t add);
void      WriteFloat_3x(uint16_t add, float value) ;
float     ReadFloat_3x(uint16_t add);
////////////////////////////////////////////////////

//////////////////////////////////////////////
void Process_data();
uint16_t CRC16(uint8_t *, uint8_t);
void modbusException(uint8_t exceptioncode);
void sendData(uint8_t *DataSend, uint8_t DataLen);

uint8_t readHoldingRegs(void);
uint8_t readInputRegs(void);
uint8_t readCoils(void);
uint8_t readInputs(void);

uint8_t writeSingleReg(void);
uint8_t writeHoldingRegs(void);
uint8_t writeSingleCoil(void);
uint8_t writeMultiCoils(void);

#define float2uint32(...) *((uint32_t*)(&__VA_ARGS__))
#define uint322float(...) *((float *)(&__VA_ARGS__))

void Write_bit_0x(uint16_t Address, bool Value) {
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  if (Value == 1) {
    Coils_0x[addByte] |= 1 << bitPosition;  // Replace that bit with 1
  } else {
    Coils_0x[addByte] &= ~(1 << bitPosition);  // Replace that bit with 0
  }
}

bool Read_bit_0x(uint16_t Address) {
  if (Address > (BUFFER_SIZE * 8 - 1)) {
    return 0;
  }
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  return ((Coils_0x[addByte] >> bitPosition) & 0X01);
}

void Write_bit_1x(uint16_t Address, bool Value) {
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  if (Value == 1) {
    Inputs_1x[addByte] |= 1 << bitPosition;  // Replace that bit with 1
  } else {
    Inputs_1x[addByte] &= ~(1 << bitPosition);  // Replace that bit with 0
  }
  return 1;  // success
}

bool Read_bit_1x(uint16_t Address) {
  if (Address > (BUFFER_SIZE * 8 - 1)) {
    return 0;
  }
  int addByte = Address / 8;
  uint16_t bitPosition = Address % 8;
  return ((Inputs_1x[addByte] >> bitPosition) & 0X01);
}
void Write_4x(uint16_t add, uint16_t value) {
  Holding_Registers_4x[add] = value;
}
void WriteFloat_4x(uint16_t add, float value) {
  Holding_Registers_4x[add]=  float2uint32(value) & 0xffff;
  Holding_Registers_4x[add+1]=float2uint32(value) >> 16 & 0xffff;
}
uint16_t Read_4x(uint16_t add) {
  return (Holding_Registers_4x[add]);
}
float ReadFloat_4x(uint16_t add) {
  uint32_t tem = Holding_Registers_4x[add+1];
  tem = tem<<16 | Holding_Registers_4x[add];
  return (uint322float(tem));
}

void Write_3x(uint16_t add, uint16_t value) {
  Input_Registers_3x[add] = value;
}
uint16_t Read_3x(uint16_t add) {
  return (Input_Registers_3x[add]);
}
void WriteFloat_3x(uint16_t add, float value) {
  Input_Registers_3x[add]=  float2uint32(value) & 0xffff;
  Input_Registers_3x[add+1]=float2uint32(value) >> 16 & 0xffff;
}
float ReadFloat_3x(uint16_t add) {
  uint32_t tem = Input_Registers_3x[add+1];
  tem = tem<<16 | Input_Registers_3x[add];
  return (uint322float(tem));
}
void serialEvent() {
  RxLength = Serial.readBytes(RxData, BUFFER_SIZE);
  if (RxLength > 0) {
    uint16_t crc_check = CRC16(RxData, RxLength - 2);
    uint16_t crc_Receive = RxData[RxLength - 2] << 8 | RxData[RxLength - 1];
    if ((RxData[0] == SLAVE_ID) && (crc_check == crc_Receive)) {
      Process_data();
    } else if ((RxData[0] == SLAVE_ID) && (crc_check != crc_Receive)) {
      modbusException(ILLEGAL_FUNCTION);
    }
  }
}


void modbusException(uint8_t exceptioncode) {
  TxData[0] = SLAVE_ID;
  TxData[1] = RxData[1] | 0x80;  //add them 1 vao MSB
  TxData[2] = exceptioncode;
  sendData(TxData, 3);
}

void Process_data() {
  switch (RxData[1]) {
    case 0x03:
      {
        readHoldingRegs();
        break;
      }
    case 0x04:
      {
        readInputRegs();
        break;
      }
    case 0x01:
      {
        readCoils();
        break;
      }
    case 0x02:
      {
        readInputs();
        break;
      }
    case 0x06:
      {
        writeSingleReg();
        break;
      }
    case 0x10:
      {
        writeHoldingRegs();
        break;
      }
    case 0x05:
      {
        writeSingleCoil();
        break;
      }
    case 0x0F:
      {
        writeMultiCoils();
        break;
      }
    default:
      {
        modbusException(ILLEGAL_FUNCTION);
        break;
      }
  }
}
void sendData(uint8_t *DataSend, uint8_t DataLen) {
  uint16_t crc = CRC16(DataSend, DataLen);
  DataSend[DataLen] = highByte(crc);  //>> 8) & 0xFF; // CRC HIGH
  DataLen++;
  DataSend[DataLen] = lowByte(crc);  // & 0xFF; // CRC LOW
  DataLen++;
  for (int i = 0; i < DataLen; i++) {
    Serial.write(DataSend[i]);
  }
}
uint8_t readHoldingRegs(void)  //Read Holding Regs 4x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

  uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);  // number to registers master has requested
  if ((numRegs < 1) || (numRegs > 125))               // maximum no. of Registers as per the PDF
  {
    modbusException(ILLEGAL_DATA_VALUE);  // send an exception
    return 0;
  }

  uint16_t endAddr = startAddr + numRegs - 1;  // end Register
  if (endAddr > BUFFER_SIZE - 1)               // end Register can not be more than 49 as we only have record of 50 Registers in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }
  // Prepare TxData buffer
  //| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
  //| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |
  TxData[0] = SLAVE_ID;     // slave ID
  TxData[1] = RxData[1];    // function code
  TxData[2] = numRegs * 2;  // Byte count
  int indx = 3;             // we need to keep track of how many bytes has been stored in TxData Buffer

  for (int i = 0; i < numRegs; i++)  // Load the actual data into TxData buffer
  {
    TxData[indx++] = (Holding_Registers_4x[startAddr] >> 8) & 0xFF;  // extract the higher byte
    TxData[indx++] = (Holding_Registers_4x[startAddr]) & 0xFF;       // extract the lower byte
    startAddr++;                                                     // increment the register address
  }

  sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
  return 1;                // success
}

uint8_t readInputRegs(void)  //Read Inputs Regs 3x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

  uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);  // number to registers master has requested
  if ((numRegs < 1) || (numRegs > 125))               // maximum no. of Registers as per the PDF
  {
    modbusException(ILLEGAL_DATA_VALUE);  // send an exception
    return 0;
  }

  uint16_t endAddr = startAddr + numRegs - 1;  // end Register
  if (endAddr > BUFFER_SIZE - 1)               // end Register can not be more than 49 as we only have record of 50 Registers in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  // Prepare TxData buffer
  //| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
  //| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |
  TxData[0] = SLAVE_ID;              // slave ID
  TxData[1] = RxData[1];             // function code
  TxData[2] = numRegs * 2;           // Byte count
  int indx = 3;                      // we need to keep track of how many bytes has been stored in TxData Buffer
  for (int i = 0; i < numRegs; i++)  // Load the actual data into TxData buffer
  {
    TxData[indx++] = (Input_Registers_3x[startAddr] >> 8) & 0xFF;  // extract the higher byte
    TxData[indx++] = (Input_Registers_3x[startAddr]) & 0xFF;       // extract the lower byte
    startAddr++;                                                   // increment the register address
  }

  sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
  return 1;                // success
}

uint8_t readCoils(void)  // Read coils 0x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Coil Address

  uint16_t numCoils = ((RxData[4] << 8) | RxData[5]);  // number to coils master has requested
  if ((numCoils < 1) || (numCoils > 2000))             // maximum no. of coils as per the PDF
  {
    modbusException(ILLEGAL_DATA_VALUE);  // send an exception
    return 0;
  }

  uint16_t endAddr = startAddr + numCoils - 1;  // Last coils address
  if (endAddr > (BUFFER_SIZE * 8 - 1))          // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  //reset TxData buffer
  memset(TxData, '\0', BUFFER_SIZE);

  // Prepare TxData buffer

  //| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
  //| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

  TxData[0] = SLAVE_ID;                                       // slave ID
  TxData[1] = RxData[1];                                      // function code
  TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);  // Byte count
  int indx = 3;                                               // we need to keep track of how many bytes has been stored in TxData Buffer

  int startByte = startAddr / 8;         // which byte we have to start extracting the data from
  uint16_t bitPosition = startAddr % 8;  // The shift position in the first byte
  int indxPosition = 0;                  // The shift position in the current indx of the TxData buffer

  // Load the actual data into TxData buffer
  for (int i = 0; i < numCoils; i++) {
    TxData[indx] |= ((Coils_0x[startByte] >> bitPosition) & 0x01)
                    << indxPosition;
    indxPosition++;
    bitPosition++;
    if (indxPosition > 7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
    {
      indxPosition = 0;
      indx++;
    }
    if (bitPosition > 7)  // if the bitposition exceeds 7, we have to increment the startbyte
    {
      bitPosition = 0;
      startByte++;
    }
  }

  if (numCoils % 8 != 0)
    indx++;                // increment the indx variable, only if the numcoils is not a multiple of 8
  sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
  return 1;                // success
}

uint8_t readInputs(void)  //Read input Regs 1x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

  uint16_t numCoils = ((RxData[4] << 8) | RxData[5]);  // number to coils master has requested
  if ((numCoils < 1) || (numCoils > 2000))             // maximum no. of coils as per the PDF
  {
    modbusException(ILLEGAL_DATA_VALUE);  // send an exception
    return 0;
  }

  uint16_t endAddr = startAddr + numCoils - 1;  // Last coils address
  if (endAddr > BUFFER_SIZE * 8 - 1)            // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  //reset TxData buffer
  memset(TxData, '\0', BUFFER_SIZE);

  // Prepare TxData buffer

  //| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
  //| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

  TxData[0] = SLAVE_ID;                                       // slave ID
  TxData[1] = RxData[1];                                      // function code
  TxData[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0);  // Byte count
  int indx = 3;                                               // we need to keep track of how many bytes has been stored in TxData Buffer

  int startByte = startAddr / 8;         // which byte we have to start extracting the data from
  uint16_t bitPosition = startAddr % 8;  // The shift position in the first byte
  int indxPosition = 0;                  // The shift position in the current indx of the TxData buffer

  // Load the actual data into TxData buffer
  for (int i = 0; i < numCoils; i++) {
    TxData[indx] |= ((Inputs_1x[startByte] >> bitPosition) & 0x01)
                    << indxPosition;
    indxPosition++;
    bitPosition++;
    if (indxPosition > 7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
    {
      indxPosition = 0;
      indx++;
    }
    if (bitPosition > 7)  // if the bitposition exceeds 7, we have to increment the startbyte
    {
      bitPosition = 0;
      startByte++;
    }
  }

  if (numCoils % 8 != 0)
    indx++;                // increment the indx variable, only if the numcoils is not a multiple of 8
  sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
  return 1;                // success
}

uint8_t writeHoldingRegs(void) {
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

  uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);  // number to registers master has requested
  if ((numRegs < 1) || (numRegs > 123))               // maximum no. of Registers as per the PDF
  {
    modbusException(ILLEGAL_DATA_VALUE);  // send an exception
    return 0;
  }

  uint16_t endAddr = startAddr + numRegs - 1;  // end Register
  if (endAddr > 49)                            // end Register can not be more than 49 as we only have record of 50 Registers in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  uint8_t indx = 7;  // we need to keep track of index in RxData
  for (int i = 0; i < numRegs; i++) {
    Holding_Registers_4x[startAddr] = (RxData[indx] << 8) | RxData[indx + 1];
    startAddr++;
    indx += 2;
  }

  // Prepare Response

  //| SLAVE_ID | FUNCTION_CODE | Start Addr | num of Regs    | CRC     |
  //| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES      | 2 BYTES |

  TxData[0] = SLAVE_ID;   // slave ID
  TxData[1] = RxData[1];  // function code
  TxData[2] = RxData[2];  // Start Addr HIGH Byte
  TxData[3] = RxData[3];  // Start Addr LOW Byte
  TxData[4] = RxData[4];  // num of Regs HIGH Byte
  TxData[5] = RxData[5];  // num of Regs LOW Byte

  sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
  return 1;             // success
}

uint8_t writeSingleReg(void)  // Write Single 4x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

  if (startAddr > BUFFER_SIZE - 1)  // The Register Address can not be more than 49 as we only have record of 50 Registers in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  /* Save the 16 bit data
	 * Data is the combination of 2 bytes, RxData[4] and RxData[5]
	 */

  Holding_Registers_4x[startAddr] = (RxData[4] << 8) | RxData[5];

  // Prepare Response

  //| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
  //| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

  TxData[0] = SLAVE_ID;   // slave ID
  TxData[1] = RxData[1];  // function code
  TxData[2] = RxData[2];  // Start Addr HIGH Byte
  TxData[3] = RxData[3];  // Start Addr LOW Byte
  TxData[4] = RxData[4];  // Reg Data HIGH Byte
  TxData[5] = RxData[5];  // Reg Data LOW  Byte

  sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
  return 1;             // success
}

uint8_t writeSingleCoil(void)  //Write single 0x = Write 1 bit 0x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Coil Address

  if (startAddr > BUFFER_SIZE * 8 - 1)  // The Coil Address can not be more than 199 as we only have record of 200 Coils in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  /* Calculation for the bit in the database, where the modification will be done */
  int startByte = startAddr / 8;         // which byte we have to start writing the data into
  uint16_t bitPosition = startAddr % 8;  // The shift position in the first byte

  if ((RxData[4] == 0xFF) && (RxData[5] == 0x00)) {
    Coils_0x[startByte] |= 1 << bitPosition;  // Replace that bit with 1
  }

  else if ((RxData[4] == 0x00) && (RxData[5] == 0x00)) {
    Coils_0x[startByte] &= ~(1 << bitPosition);  // Replace that bit with 0
  }

  // Prepare Response

  //| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
  //| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

  TxData[0] = SLAVE_ID;   // slave ID
  TxData[1] = RxData[1];  // function code
  TxData[2] = RxData[2];  // Start Addr HIGH Byte
  TxData[3] = RxData[3];  // Start Addr LOW Byte
  TxData[4] = RxData[4];  // Coil Data HIGH Byte
  TxData[5] = RxData[5];  // Coil Data LOW  Byte

  sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
  return 1;             // success
}

uint8_t writeMultiCoils(void)  // Write 0x
{
  uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Coil Address

  uint16_t numCoils = ((RxData[4] << 8) | RxData[5]);  // number to coils master has requested
  if ((numCoils < 1) || (numCoils > 1968))             // maximum no. of coils as per the PDF
  {
    modbusException(ILLEGAL_DATA_VALUE);  // send an exception
    return 0;
  }

  uint16_t endAddr = startAddr + numCoils - 1;  // Last coils address
  if (endAddr > 199)                            // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
  {
    modbusException(ILLEGAL_DATA_ADDRESS);  // send an exception
    return 0;
  }

  /* Calculation for the bit in the database, where the modification will be done */
  int startByte = startAddr / 8;         // which byte we have to start writing the data into
  uint16_t bitPosition = startAddr % 8;  // The shift position in the first byte
  int indxPosition = 0;                  // The shift position in the current indx of the RxData buffer

  int indx = 7;  // we need to keep track of index in RxData

  // Modify the bits as per the Byte received
  for (int i = 0; i < numCoils; i++) {
    if (((RxData[indx] >> indxPosition) & 0x01) == 1) {
      Coils_0x[startByte] |= 1 << bitPosition;  // replace that bit with 1
    } else {
      Coils_0x[startByte] &= ~(1 << bitPosition);  // replace that bit with 0
    }

    bitPosition++;
    indxPosition++;

    if (indxPosition > 7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
    {
      indxPosition = 0;
      indx++;
    }
    if (bitPosition > 7)  // if the bitposition exceeds 7, we have to increment the startbyte
    {
      bitPosition = 0;
      startByte++;
    }
  }

  // Prepare Response

  //| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
  //| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

  TxData[0] = SLAVE_ID;   // slave ID
  TxData[1] = RxData[1];  // function code
  TxData[2] = RxData[2];  // Start Addr HIGH Byte
  TxData[3] = RxData[3];  // Start Addr LOW Byte
  TxData[4] = RxData[4];  // num of coils HIGH Byte
  TxData[5] = RxData[5];  // num of coils LOW  Byte

  sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
  return 1;             // success
}

/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40
};

//unsigned uIndex ; /* will index into CRC lookup table */
static char auchCRCLo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
  0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
  0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
  0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
  0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
  0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
  0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
  0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
  0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
  0x40
};
uint16_t CRC16(uint8_t *puchMsg, uint8_t usDataLen) {
  unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
  unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
  unsigned uIndex;               /* will index into CRC lookup table */
  //while (usDataLen––) /* pass through message buffer */
  while (usDataLen--) {
    uIndex = uchCRCHi ^ *puchMsg++; /* calculate the CRC */
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];
  }
  return (uchCRCHi << 8 | uchCRCLo);
}
void delaysecond(uint8_t time_) {
  while (time_-- > 0) {
    delay(245);
    serialEvent();
    delay(245);
    serialEvent();
    delay(245);
    serialEvent();
    delay(245);
    serialEvent();
  }
}
