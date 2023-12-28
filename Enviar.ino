#include <mcp2515.h>
#include <SPI.h>
#include <EEPROM.h>


int valorkm;
int valorrpm;
int valorpo;
int valortm;

int km;
int rpm;
int po;
int tm;

struct can_frame canMsg;
MCP2515 mcp2515(10);


struct EepromData{
  short header;
  long hodometer;
  long hourmeter;
  long litrometer;
  long fuelLevel;
};



EepromData telemetryData;

struct PGNs {
    unsigned int pgn;
    unsigned int spn;
    unsigned int len;
    float pos;
    float res;
    int ost;
    float val;
    char* name;
    float inc;
    int type;
};

#define MAX_PGN 20
unsigned long _CANID_MASK = 0b10000000000000000000000000000000;
int xPgns = 0;
PGNs _pgns[MAX_PGN];
int x = 0;

int count = 0;

void setup() {
    Serial.begin(9600);
    Serial.println(F("CAN SIMULATOR STARTING..."));

    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();    
    
    EEPROM.get(0, telemetryData);

    if(!checkFirstAddressValue(telemetryData.header)){    
      Serial.print("Clear EEPROM!");
      telemetryData.hodometer = 0;      
      telemetryData.hourmeter = 0;
      telemetryData.litrometer = 0;
      telemetryData.fuelLevel = 100;
    }

}

void loop() {
    valorkm = analogRead(A2);
    km = valorkm * (200.0 / 1023.0); //velocidade

    valorrpm = analogRead(A3);
    rpm = valorrpm * (3000.0 / 1023); //rpm

    valortm = analogRead(A1);
    tm = valortm * (150.0 / 1023.0); //temperatura motor

    valorpo = analogRead(A4);
    po = valorpo * (200.0 / 1023.0); //pressão do oleo

    // Zera os PGNs antes de adicionar novos dados
    xPgns = 0;

    addPGN(65253, 247,   4, 1, 0.05,   0,  500,  (char*) "65253-HOURMETER",       0.1,  1 );   
    addPGN(65262, 110,   1, 1, 1,     -40, tm,  (char*) "65262-LIQUID TEMP",     15,   1 );    //pot
    addPGN(61444, 190,   2, 4, 0.125,  0,  rpm,  (char*) "61444-ENGINE RPM",      50,   1 );   //pot
    addPGN(65263, 100,   1, 4, 4,      0,  po,  (char*) "65263-OIL PRESSURE",    50,   1 );    //pot 
    addPGN(65276, 96,    1, 2, 0.4,    0,  050,  (char*) "65276-FUEL LEVEL",      05,   1 );
    addPGN(65265, 84,    2, 2, 0.0039,  0,  km,    (char*) "65265-VEHICLE SPEED",   05,   1 ); //pot        
    addPGN(65257, 250,   4, 5, 0.5,    0,  005,  (char*) "65257-TOTAL FUEL",      05,   1 );      
    addPGN(65132, 1624,  2, 7, 1,      0,  80,   (char*) "65132-Tachograph",      05,   1 ); 

    
    sendPGNs();

    delay(100);   

    count++;
    telemetryData.hodometer++;

    if(count % 50 == 0){          
      telemetryData.hodometer += km;
      Serial.print("hod = ");
      Serial.println(telemetryData.hodometer);
    }

    if(count % 250 == 0){    
      telemetryData.header = 10;     
      Serial.print("Gravei na EPROM: hod->");
      Serial.println(telemetryData.hodometer);
      EEPROM.put(0, telemetryData);
    }
    
}

void sendPGNs() {
    for (int i = 0; i < xPgns; i++) {
        sendGenericPGN(_pgns[i]);
        delay(10);
    }
}

void sendGenericPGN(PGNs _pgn) {
    struct can_frame canMsg1;

    //Serial.print(F("PGN----------------------------------------------------------------------------: "));
    //Serial.println(_pgn.pgn);

    canMsg1.can_id  = getCanId(_pgn.pgn);
    canMsg1.can_dlc = 8;

    canMsg1.data[0] = 0xFF;
    canMsg1.data[1] = 0xFF;
    canMsg1.data[2] = 0xFF;
    canMsg1.data[3] = 0xFF;
    canMsg1.data[4] = 0xFF;
    canMsg1.data[5] = 0xFF;
    canMsg1.data[6] = 0xFF;
    canMsg1.data[7] = 0xFF;

    if (_pgn.type == 1) { // FULL BYTE
        float value  = _pgn.val - _pgn.ost;
        int   intVal = value / _pgn.res;
        for (int i = 0; i < _pgn.len; ++i) {
            byte b  = 0b11111111 & intVal;
            intVal  = intVal >> 8;
            int pos = ((int)_pgn.pos - 1) + i;
            canMsg1.data[pos] = b;
        }
    }

    mcp2515.sendMessage(&canMsg1);
}

void addPGN (
              unsigned int    pgn,
              unsigned int    spn,
              unsigned int    len,
              float           pos,
              float           res,
              int             ost,
              float           val,
              char*           name,
              float           inc,
              int             type) {
    _pgns[xPgns].pgn    = pgn;
    _pgns[xPgns].spn    = spn;
    _pgns[xPgns].len    = len;
    _pgns[xPgns].pos    = pos;
    _pgns[xPgns].res    = res;
    _pgns[xPgns].ost    = ost;
    _pgns[xPgns].val    = val;
    _pgns[xPgns].name   = name;
    _pgns[xPgns].inc    = inc;
    _pgns[xPgns].type   = type;
    xPgns++;
}

unsigned long getCanId(int _pgn) {
    unsigned long pgn   = 0b1111111111111111 & _pgn;
    unsigned long mPgn  = pgn << 8;
    unsigned long canId = _CANID_MASK | mPgn;

/*
    Serial.print(F("canId: "));
    Serial.println(canId);

    Serial.print(F("pgn: "));
    Serial.println(pgn);
  */  

    return canId;
}



bool checkFirstAddressValue(short data) {
  return (data == 10);
}