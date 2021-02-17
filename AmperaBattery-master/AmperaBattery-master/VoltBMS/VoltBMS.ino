/*
  Chevy Volt BMS slaves decode

  030618

  Tom de Bree
  D. Maguire
*/


#include <can_common.h>
#include <due_can.h>
#include <due_wire.h>
#include <DueTimer.h>

//Can variables
uint32_t can0Speed = 125000;
uint32_t tlast = 0;

uint16_t Cell[97]; //cells, raw 0.00125/V
uint16_t Temp[24]; // temperatures 0.0556/C and -27.778
uint16_t celllow, cellhigh, templow, temphigh;

#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

  // Initialize CAN ports
  if (Can0.begin(can0Speed, 255)) //can1 external bus
  {
    Serial.println("Using CAN1 - initialization completed.\n");
  }
  else Serial.println("CAN1 initialization (sync) ERROR\n");

  int filter;
  //extended
  for (filter = 0; filter < 3; filter++)
  {
    Can0.setRXFilter(filter, 0, 0, true);
  }
  //standard
  for (int filter = 3; filter < 7; filter++)
  {
    Can0.setRXFilter(filter, 0, 0, false);
  }

}

void loop()
{
  // put your main code here, to run repeatedly:
  CAN_FRAME incoming;

  if (Can0.available())
  {
    Can0.read(incoming);
    candecode(incoming);
  }

  if (tlast <  (millis() - 500)) // 2hz loop
  {
    tlast = millis();
    printcells();
  }
}

void printcells()
{
  int x = 0;
  cellhigh = 0;
  celllow = 4000; //5V / 0.00125
  Serial.println();
  Serial.println("Cell Voltages");
  for (int y = 1; y < 97; y++)
  {
    Serial.print(Cell[y] * 0.00125, 2);
    Serial.print(" | ");
    if (Cell[y] < celllow)
    {
      celllow = Cell[y];
    }
    if (Cell[y] > cellhigh)
    {
      cellhigh = Cell[y];
    }
    x++;
    if (x > 7)
    {
      x = 0;
      Serial.println();
    }
  }
  Serial.println();
  Serial.print("Lowest Cell: ");
  Serial.print(celllow * 0.00125, 2);
  Serial.print("  Highest Cell: ");
  Serial.print(cellhigh * 0.00125, 2);
  Serial.println();

  templow = 5000;
  temphigh = 0;
  x = 0;
  for (int y = 1; y < 17; y++)
  {
    Serial.print((Temp[y] * 0.0556) - 27.778, 2);
    Serial.print(" | ");
    if (Temp[y] < templow)
    {
      templow = Temp[y];
    }
    if (Temp[y] > temphigh)
    {
      temphigh = Temp[y];
    }
    x++;
    if (x > 7)
    {
      x = 0;
      Serial.println();
    }
  }
  Serial.println();
  Serial.print("Lowest Temp: ");
  Serial.print((templow * 0.0556) - 27.778, 2);
  Serial.print("  Highest Temp: ");
  Serial.print((temphigh * 0.0556) - 27.778, 2);
}


void candecode(CAN_FRAME & frame)
{
  int x = 0;
  switch (frame.id)
  {
    case 0x460: //Module 1 cells 1-3
      Cell[1] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[2] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[3] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x470: //Module 1 cells 4-6
      Cell[4] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[5] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[6] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x461: //Module 1 cells 7-10
      Cell[7] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[8] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[9] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[10] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x471: //Module 1 cells 11-14
      Cell[11] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[12] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[13] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[14] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x462: //Module 1 cells 15-18
      Cell[15] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[16] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[17] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[18] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x472: //Module 1 cells 19-22
      Cell[19] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[20] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[21] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[22] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x463: //Module 1 cells 23-26
      Cell[23] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[24] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[25] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[26] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x473: //Module 1 cells 27-30
      Cell[27] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[28] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[29] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[30] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x464: //Module 2 cells 31-34
      Cell[31] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[32] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[33] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[34] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x474: //Module 2 cells 35-38
      Cell[35] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[36] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[37] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[38] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x465: //Module 2 cells 39-42
      Cell[39] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[40] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[41] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[42] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x475: //Module 2 cells 43-46
      Cell[43] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[44] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[45] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[46] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x466: //Module 2 cells 47-50
      Cell[47] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[48] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[49] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[50] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x476: //Module 2 cells 51-54
      Cell[51] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[52] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[53] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[54] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x468: //Module 3 cells 55-58
      Cell[55] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[56] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[57] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[58] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x478: //Module 3 cells 59-62
      Cell[59] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[60] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[61] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[62] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x469: //Module 3 cells 63-66
      Cell[63] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[64] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[65] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[66] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x479: //Module 3 cells 67-70
      Cell[67] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[68] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[69] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[70] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x46A: //Module 3 cells 71-74
      Cell[71] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[72] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[73] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[74] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;
    case 0x47A: //Module 3 cells 75-78
      Cell[75] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[76] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[77] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      Cell[78] = (frame.data.bytes[6] & 0x0F) << 8 + frame.data.bytes[7];
      break;

    case 0x46C: //Module 4 cells 79-81
      Cell[79] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[80] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[81] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x47C: //Module 4 cells 82-84
      Cell[82] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[83] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[84] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x46D: //Module 4 cells 85-87
      Cell[85] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[86] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[87] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x47D: //Module 4 cells 88-90
      Cell[88] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[89] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[90] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x46E: //Module 4 cells 91-93
      Cell[91] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[92] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[93] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;
    case 0x47E: //Module 4 cells 94-96
      Cell[94] = (frame.data.bytes[0] & 0x0F) << 8 + frame.data.bytes[1];
      Cell[95] = (frame.data.bytes[2] & 0x0F) << 8 + frame.data.bytes[3];
      Cell[96] = (frame.data.bytes[4] & 0x0F) << 8 + frame.data.bytes[5];
      break;

    case 0x7E0: //Module 1 Temp 1
      Temp[1] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
    case 0x7E1: //Module 1 Temp 2
      Temp[2] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
    case 0x7E2: //Module 1 Temp 3-4
      Temp[3] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      Temp[4] = (frame.data.bytes[2] & 0x03) << 8 + frame.data.bytes[3];
      break;
    case 0x7E3: //Module 1 Temp 5
      Temp[5] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;

    case 0x7E4: //Module 2 Temp 6
      Temp[6] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
    case 0x7E5: //Module 2 Temp 7-8
      Temp[7] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      Temp[8] = (frame.data.bytes[2] & 0x03) << 8 + frame.data.bytes[3];
      break;
    case 0x7E6: //Module 2 Temp 9
      Temp[9] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;

    case 0x7E8: //Module 3 Temp 10
      Temp[10] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
    case 0x7E9: //Module 3 Temp 11-12
      Temp[11] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      Temp[12] = (frame.data.bytes[2] & 0x03) << 8 + frame.data.bytes[3];
      break;
    case 0x7EA: //Module 3 Temp 13
      Temp[13] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;

    case 0x7EC: //Module 4 Temp 14
      Temp[14] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
    case 0x7ED: //Module 4 Temp 15
      Temp[15] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
    case 0x7EE: //Module 4 Temp 16
      Temp[16] = (frame.data.bytes[0] & 0x03) << 8 + frame.data.bytes[1];
      break;
  }
}
