//
// Created by gregory on 28/09/2020.
//

#include "Arduino.h"
#include <mcp2515.h>
#include "j1939_converter.h"
#include "j1939_spn.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>


/**
 *
 * @param CAN
 */
j1939Converter::j1939Converter( SoftwareSerial *rs485 )
{
    this->_rs485 = rs485;
}


/**
 *
 */
void j1939Converter::setup(  )
{
    this->deviceAddr = EEPROM.read(0);
    if(!this->deviceAddr || this->deviceAddr == 255) {
        Serial.println(F("| - Invalid Device Address...                                                                |"));
        this->help();
        return;
    }
    else {
        Serial.print  (F("| - Device Address: "));
        Serial.println( this->deviceAddr );
    }
    if(this->loadPgns() > 0) {
        Serial.println(F("| - PgnList is EMPTY!!! "));
        this->help();
        return;
    }
}


/**
 *
 */
int j1939Converter::help()
{
    /*
    Serial.println(F("+--------------------------------------------------------------------------------------------+"));
    Serial.println(F("| -- CONFIGURATION                                                                            |"));
    Serial.println(F("|                                                                                             |"));
    Serial.println(F("| Set DevAddr         : CS|setAddr|<addr>                                                     |"));
    Serial.println(F("| Clear EEPROM        : CS|cEEPROM|                                                           |"));
    Serial.println(F("| Add PGN             : CS|j1939-addPGN|devAddr|pgn,spn,pos,len,type,resolution,offset        |"));
    Serial.println(F("|  65253-HOURMETER    : CS|j1939-addPGN|1|65253,247,1,4,2,0.05,0                              |"));
    Serial.println(F("|  61444-ENGINE RPM   : CS|j1939-addPGN|1|61444,190,4,2,2,0.125,0                             |"));
    Serial.println(F("|  65265-VEHICLE SPEED: CS|j1939-addPGN|1|65265,84,2,2,2,0.003,0                              |"));
    Serial.println(F("|  65276-FUEL LEVEL   : CS|j1939-addPGN|1|65276,96,2,1,2,0.4,0                                |"));
    Serial.println(F("|  65257-TOTAL FUEL   : CS|j1939-addPGN|1|65257,250,5,4,2,0.5,0                               |"));
    Serial.println(F("|                                                                                             |"));
    Serial.println(F("+--------------------------------------------------------------------------------------------+"));

    return 0;
    /*
     *         void addPGN(unsigned int    pgn,
                    unsigned int    spn,
                    unsigned int    len,
                    float           pos,
                    float           res,
                    int             ost,
                    float           val,
                    char*           name,
                    float           inc,
                    int             type);
     *
     _LCD.addPGN(65253, 247,   4, 1, 0.05,   0,  500,  (char*) "65253-HOURMETER",      0.1,  1 );
    _LCD.addPGN(65262, 110,   1, 1, 1,     -40, 030,  (char*) "65262-LIQUID TEMP",    15,   1 );
    _LCD.addPGN(61444, 190,   2, 4, 0.125,  0,  600,  (char*) "61444-ENGINE RPM",     50,   1 );
    _LCD.addPGN(65263, 100,   1, 4, 4,      0,  200,  (char*) "65263-OIL PRESSURE",   50,   1 );
    _LCD.addPGN(65276, 96,    1, 2, 0.4,    0,  050,  (char*) "65276-FUEL LEVEL",     05,   1 );
    _LCD.addPGN(65265, 84,    2, 2, 0.003,  0,  005,  (char*) "65265-VEHICLE SPEED",  05,   1 );
    _LCD.addPGN(65318, 123,   2, 1, 1,      0,  300,  (char*) "65318-NH-INDUSTRIA",   20,   1 );
    _LCD.addPGN(65397, 123,   1, 2, 1,      0,  0,    (char*) "65397-NH-ROLO",        8,    1 );
     * */

}


/**
 *
 */
void j1939Converter::canRead(  struct can_frame *frame  )
{
    if(!this->deviceAddr || this->deviceAddr == 255) {
        return;
    }
    if(!this->nPgns || this->nPgns < 1) {
        return;
    }

    unsigned long canId = frame->can_id;
    unsigned char buf[8];
    unsigned char len = frame->can_dlc;
    unsigned int _pgn = 0;

    canId   = 0b00000111111111111111100000000 & canId;
    canId   = canId >> 8;
    _pgn    = canId;

    for (int i = 0; i<frame->can_dlc; i++) {
        buf [i] = frame->data[i];
    }
    for (short i = 0; i < this->nPgns; i++) {
        if(this->spnList[i].pgn == _pgn) {
            this->translatePGN( &this->spnList[i], buf );
        }
    }
}


/**
 *
 */
void j1939Converter::translatePGN( SPNs *pgn, unsigned char buf[8] )
{
    // DATA TYPE 2 = BYTE
    if(pgn->dataType == 2) {

        Serial.println(pgn->pgn);

        unsigned long pgnValue = 0b00000000000000000000000000000000;
        short i = (pgn->length + pgn->pos) -1;
        for (i; i > pgn->pos; --i) {
            pgnValue = pgnValue | buf[i-1];
            pgnValue = pgnValue << 8;
        }
        pgnValue = pgnValue | buf[i-1];
        pgn->fValue = pgnValue * pgn->resolution;
        pgn->fValue = pgn->fValue + pgn->offset;
    }
}


/**
 *
 * @param _rs485
 */
void j1939Converter::serialListener()
{
    String inputString = "";
    bool stringComplete = false;
    char inChar;

    while (Serial.available()) {
        inChar = (char) Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
            break;
        }
        inputString += inChar;
        delay(1); // SEM ESTE DELAY NÃO FUNCIONA A CONCATENAÇÃO DA STRING -- provavelmente um BUG do arduino
    }

    if(stringComplete) {
        switch ( this->parseFrame( inputString ) ) {
            case 0  : this->executeCommand(); break;
            case 1  : this->help(); break;
            default : this->help(); break;
        }
    }
}


/**
 *
 * @param _rs485
 */
void j1939Converter::rs485Parse( String frame )
{
    if(!this->deviceAddr) return;
    if (this->parseFrame( frame ) > 0) return;
    if (this->checkDeviceAddress() > 0) return;
    if (this->crcValidate() > 0) return;
    this->sendACK(this->_frameParse[1]);
    this->executeCommand();
}


/**
 *
 * @param _frame
 * @param pFrame
 * @return
 */
int j1939Converter::parseFrame ( String _frame )
{
    // CHECK PROTOCOL HEADER
    if (_frame[0] != 'C' || _frame[1] != 'S') return 1;

    // LIMPAR O FRAME PARSE
    for (int i = 0; i < MAX_FRAME ; ++i) {
        this->_frameParse[i] = "\0";
    }

    this->frameL = 0;
    for (int i = 0; i < _frame.length(); ++i) {
        if (_frame[i] == '|')  this->frameL++;
        else this->_frameParse[this->frameL] += _frame[i];
    }
    return 0;
}


/**
 *
 * @return
 */
int j1939Converter::executeCommand()
{
    int cmd = -1;
            if ( this->_frameParse[1] == "setAddr" )        cmd = this->setDeviceAddr();
    else    if ( this->_frameParse[1] == "help" )           cmd = this->help();
    else    if ( this->_frameParse[1] == "cEEPROM" )        cmd = this->cleanEEPROM();
    else    if ( this->_frameParse[1] == "start" )          cmd = this->startProtocol();
    else    if ( this->_frameParse[1] == "j1939-addPGN" )   cmd = this->addPGN();
    else    if ( this->_frameParse[1] == "j1939-getData" )  cmd = this->getPgnData();

    switch (cmd) {
        case 0  :   Serial.println  (F("| - Success Command: ")); break;
        case 1  :   Serial.println  (F("| - Fail Command: ")); break;
        default :   Serial.print    (F("| - Invalid Command: "));
                    Serial.println  (this->_frameParse[1] ); break;
    }

    return cmd;
}

int  j1939Converter::setDeviceAddr()
{
    int addr = this->_frameParse[2].toInt();
    if ( addr < 1 || addr > 254 ) {
        Serial.print(F("| - invalid Device Address: "));
        return 1;
    }

    this->deviceAddr = addr;
    EEPROM.write(0, this->deviceAddr);
    return 0;
}

int  j1939Converter::cleanEEPROM()
{
    Serial.println(F("| - cleanEEPROM: "));
    for (short i = 0; i < EEPROM.length(); ++i) {
        EEPROM.write(i, 0xFF);
    }
    return 0;
}

int j1939Converter::checkDeviceAddress()
{
    if( this->deviceAddr == this->_frameParse[2].toInt() ) {
        return 0;
    }
    return 1;
}

void j1939Converter::sendNACK( String reason )
{
    this->_frame = "CS|NACK|$1|$2|";
    this->_frame.replace("$1", String(this->deviceAddr));
    this->_frame.replace("$2", String(reason));
    this->rs485Print();
}

void j1939Converter::sendACK( String func )
{
    this->_frame = "CS|ACK|$1|$2|";
    this->_frame.replace("$1", String(this->deviceAddr));
    this->_frame.replace("$2", String(func));
    this->rs485Print();
}

int j1939Converter::startProtocol()
{
    this->_frame = "CS|start|$1|$2|";
    this->_frame.replace("$1", String(this->deviceAddr));
    this->_frame.replace("$2", "j1939-serialBus");
    this->rs485Print();
}

int j1939Converter::getPgnData()
{
    for (short i = 0; i < this->nPgns; i++) {
        this->_frame  = "CS|j1939-pgnData|$1|$2|$3|$4|";
        this->_frame.replace("$1", (String) this->deviceAddr);
        this->_frame.replace("$2", (String) this->spnList[i].pgn);
        this->_frame.replace("$3", (String) this->spnList[i].spn);
        this->_frame.replace("$4", (String) this->spnList[i].fValue);
        this->rs485Print();
    }

    return 0;
}

void j1939Converter::rs485Print()
{
    uint16_t crc = this->getCRC16_();
    digitalWrite(6, HIGH);
    this->_rs485->print(this->_frame);
    this->_rs485->println(crc);
    digitalWrite(6, LOW);
    delay(100);
}


/**
 *
 */
int j1939Converter::loadPgns()
{
    this->nPgns = 0;
    for(short i = 0; i < MAX_PGN; i++) {
        EEPROM.get( ((i+1) * PGN_SIZE), this->spnList[this->nPgns]  );
        if(this->spnList[this->nPgns].pgn <= 00000) continue;
        if(this->spnList[this->nPgns].pgn >= 65535) continue;
        Serial.print(F("| - PGN: "));
        Serial.print( this->spnList[this->nPgns].pgn );
        Serial.print(F(", SPN: "));
        Serial.print( this->spnList[this->nPgns].spn );
        Serial.print(F(", POS: "));
        Serial.print( this->spnList[this->nPgns].pos );
        Serial.println();
        this->nPgns++;
    }

    if (this->nPgns == 0) return 1;
    return 0;
}




/**
 *
 * @return
 */
int  j1939Converter::addPGN()
{
    if ( this->nPgns >= MAX_PGN) {
        Serial.println(F("| - ERROR: Maximum number of pgns..."));
        return 1;
    }

    String _params[] = {"","","","","","",""};
    short p = 0;

    for (short i = 0; i < this->_frameParse[3].length() ; ++i) {
        if(this->_frameParse[3][i] == ',') p++;
        else _params[p] += this->_frameParse[3][i];
    }

    if(_params[0].toInt() <= 0 || _params[0].toInt() >= 65535)
        return 2;

    this->spnList[this->nPgns].pgn          = _params[0].toInt();
    this->spnList[this->nPgns].spn          = _params[1].toInt();
    this->spnList[this->nPgns].pos          = _params[2].toFloat();
    this->spnList[this->nPgns].length       = _params[3].toInt();
    this->spnList[this->nPgns].dataType     = _params[4].toInt();
    this->spnList[this->nPgns].resolution   = _params[5].toFloat();
    this->spnList[this->nPgns].offset       = _params[6].toInt();
    this->spnList[this->nPgns].fValue       = 0;

    EEPROM.put( ((this->nPgns+1) * PGN_SIZE), this->spnList[this->nPgns] );
    this->nPgns++;
    return 0;
}

/**
 *
 *
 * @param _frame
 * @return
 */
uint16_t j1939Converter::getCRC16( String _frame )
{
    uint16_t crc = 0xFFFF;
    uint16_t x;

    for (short i = 0; i < _frame.length(); i++) {
        x   = ((crc >> 8) ^ int(_frame[i]) ) & 0xFF;
        x  ^= x >> 4;
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF;
    }
    return crc;
}

/**
 *
 *
 * @param _frame
 * @return
 */
uint16_t j1939Converter::getCRC16_()
{
    uint16_t crc = 0xFFFF;
    uint16_t x;

    for (short i = 0; i < this->_frame.length(); i++) {
        x   = ((crc >> 8) ^ int(this->_frame[i]) ) & 0xFF;
        x  ^= x >> 4;
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF;
    }
    return crc;
}



int j1939Converter::crcValidate()
{
    if(this->_frameParse[this->frameL-1].length() < 1) {
        this->sendNACK("invalidCRCLength");
        return 1;
    }

    String frameWithoutCRC = "";
    for (short i = 0; i < this->frameL; ++i) {
        if (this->_frameParse[i] != "\0") {
            frameWithoutCRC += this->_frameParse[i];
            frameWithoutCRC += "|";
        }
        else break;
    }

    this->_frame = frameWithoutCRC;

    uint16_t crc    = this->getCRC16_();
    uint16_t crcx   = (uint16_t) this->_frameParse[this->frameL].toInt();

    if(crcx != crc) {
        this->sendNACK("invalidCRC");
        return 1;
    }

    return 0;
}
