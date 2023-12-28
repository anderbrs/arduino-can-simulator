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
  short header;      //o valor 10 indica que ha dados validos na eeprom
  long hodometer;    //hodometro em metros
  long hourmeter;    //horimetro em segundos
  long litrometer;   //consumo de combustível em mililitros
  long fuelLevel;    //nivel de combustivel em % * 100 (ex: 50% = 5000)   
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

    if(telemetryData.header != 10){   

      Serial.println("#############");   
      Serial.println("Clear EEPROM!");
      Serial.println("#############");   

      telemetryData.hodometer = 0;      
      telemetryData.hourmeter = 0;
      telemetryData.litrometer = 0;
      telemetryData.fuelLevel = 10000;
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

    // O QUINTO PARAMETRO É O DIVISOR ANTES DO ENVIO!

    addPGN(65253, 247,   4, 1, 180,     0,  telemetryData.hourmeter,  (char*) "65253-HOURMETER",0.1,  1 );   
    addPGN(65262, 110,   1, 1, 1,       0,  tm,  (char*) "65262-LIQUID TEMP", 15, 1 );    
    addPGN(61444, 190,   2, 4, 0.125,   0,  rpm,  (char*) "61444-ENGINE RPM", 50, 1 );   
    addPGN(65263, 100,   1, 4, 4,       0,  po,  (char*) "65263-OIL PRESSURE", 50, 1 ); 
    addPGN(65276, 96,    1, 2, 40,      0,  telemetryData.fuelLevel,  (char*) "65276-FUEL LEVEL",05, 1 );
    addPGN(65265, 84,    2, 2, 0.0039,  0,  km,    (char*) "65265-VEHICLE SPEED", 05, 1 );
    addPGN(65257, 250,   4, 1, 500,     0,  telemetryData.litrometer,  (char*) "65257-TOTAL FUEL",05, 1 );      
    addPGN(65217, 917,   4, 1, 5,       0,  telemetryData.hodometer,  (char*) "65217-HODOMETER", 0.1, 1 );
    
    sendPGNs();
    count++;

    if(count % 10 == 0){       
        updateTelemetryData();       
    }

    if(count % 300 == 0){    
      telemetryData.header = 10; 
      Serial.println("GRAVANDO EEPROM");     
      EEPROM.put(0, telemetryData);
    }

    delay(100);    
}

void updateTelemetryData(){

    //converte velocidade (km) para m/s e acumula no hodometro
    telemetryData.hodometer += (km * 10) / 36;

    //se rpm > 400 acumula 1 segundo no hourmeter
    if(rpm > 400){
        telemetryData.hourmeter += 1;
    }

    //acumala rpm/100 em mililitros no litrometer
    telemetryData.litrometer += rpm / 100;

    //desconta rpm/100 no fuelLevel
    telemetryData.fuelLevel -= rpm / 100;

    // se fuelLevel < 0, reiniia em 10000
    if(telemetryData.fuelLevel < 0){
        telemetryData.fuelLevel = 10000;
    }

    printValues();   
 
}

void printValues(){
      //printa os valores dos potenciometros
      Serial.print(" km=");  
      Serial.print(km);
      Serial.print(" rpm=");
      Serial.print(rpm);
      Serial.print(" po=");
      Serial.print(po);
      Serial.print(" tm=");
      Serial.print(tm);      
      
      Serial.print(" hod=");
      Serial.print(telemetryData.hodometer);
      Serial.print(" hour=");
      Serial.print(telemetryData.hourmeter);
      Serial.print(" litro=");
      Serial.print(telemetryData.litrometer);
      Serial.print(" fuel=");
      Serial.print(telemetryData.fuelLevel);

      Serial.println();
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
