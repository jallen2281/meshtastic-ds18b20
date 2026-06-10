#ifndef DS2438_h
#define DS2438_h

/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2010 Guilherme Barros
*/


#include <OneWire.h>

#define DS2438MODEL 0x26

#define WRITESCRATCH   0x4E
#define COPYSCRATCH    0x48
#define READSCRATCH    0xBE
#define RECALLSCRATCH  0xB8
#define CONVERTT       0x44
#define CONVERTV       0xB4

// Scratchpad locations
#define STATUS    0
#define TEMP_LSB  1
#define TEMP_MSB  2
#define VOLT_LSB  3
#define VOLT_MSB  4
#define CURR_LSB  5
#define CURR_MSB  6
#define THRESH    7


typedef uint8_t DeviceAddress[8];

class ds2438
{
  public:
    ds2438(OneWire*, uint8_t*);
    float readTempF();
    float readTempC();
    float readVolt();
    uint8_t readSetup();
    bool writeSetup(uint8_t);
    float readHum();

  private:
    OneWire* _wire;
    uint8_t* _deviceAddress;
    typedef uint8_t ScratchPad[9];
    bool _parasite;
    void _readMem(uint8_t*);
};
#endif
