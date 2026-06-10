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

#include "DS2438.h"

extern "C" {
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
}

ds2438::ds2438(OneWire* _oneWire, uint8_t* deviceAddress)
{
  _wire = _oneWire;
  _deviceAddress = deviceAddress;
}

bool ds2438::writeSetup(uint8_t config)
{
  //write config to scratchpad
  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(WRITESCRATCH, 0);
  _wire->write(0x00, 0); //write to first block
  _wire->write(config, 0);

  //confirm good write
  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(READSCRATCH, 0);
  _wire->write(0x00, 0);

  uint8_t compare = _wire->read();

  if ( compare != config) return 0;

  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(COPYSCRATCH, 0);
  _wire->write(0x00, 0);
  delay(20);
  
  return 1;
}

uint8_t ds2438::readSetup()
{
  ScratchPad _scratchPad;
  _readMem(_scratchPad);

  return _scratchPad[0];
}

float ds2438::readHum()
{
  // humidity can be calculated via two methods with the HIH-4010
  //VOUT=(VSUPPLY)(0.0062(sensor RH) + 0.16), typical at 25 ºC
  //((vout / vsupply) - 0.16)/0.0062 = RH @ 25 ºC
  //or temp compensated:
  //True RH = (Sensor RH)/(1.0546 – 0.00216T), T in ºC

  float nowTemp = readTempC();

  writeSetup(0x0F); // read source voltage for formula
  float sourceVolt = readVolt();

  writeSetup(0x00); // back to humidity voltage
  float sensorVolt = readVolt();

  float stdHum = ((sensorVolt / sourceVolt) - 0.16) / 0.0062;
  float trueHum = stdHum / (1.0546 - (0.00216 * nowTemp)); 

  return trueHum;
}

float ds2438::readTempC()
{
  //override for now, plsfixkthx
  _parasite = 1;

  //request temp conversion
  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(CONVERTT, _parasite);
  delay(20);

  //copy data from eeprom to scratchpad & read scratchpad
  ScratchPad _scratchPad;
  _readMem(_scratchPad);
  

  //return tempC (ignore 3 lsb as they are always 0);
  int16_t rawTemp = ( ((int16_t)_scratchPad[TEMP_MSB]) << 5) | (_scratchPad[TEMP_LSB] >> 3);

  return (float)rawTemp * 0.03125;
}

float ds2438::readTempF()
{
  return (readTempC() * 1.8) + 32;
}

float ds2438::readVolt()
{
  //override for now, plsfixkthx
  _parasite = 1;

  //request temp conversion
  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(CONVERTV, _parasite);
  delay(10);

  //copy data from eeprom to scratchpad & read scratchpad
  ScratchPad _scratchPad;
  _readMem(_scratchPad);

  //return tempC (ignore 3 lsb as they are always 0);
  int16_t rawVolt = ( ((int16_t)_scratchPad[VOLT_MSB]) << 8) | _scratchPad[VOLT_LSB];

  return (float)rawVolt * 0.01;
}


void ds2438::_readMem(uint8_t* _scratchPad)
{
  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(RECALLSCRATCH);
  _wire->write(0x00); // starting at address 0x00

  _wire->reset();
  _wire->select(_deviceAddress);
  _wire->write(READSCRATCH);
  _wire->write(0x00); // starting at address 0x00

  _scratchPad[STATUS] = _wire->read();
  _scratchPad[TEMP_LSB] = _wire->read();
  _scratchPad[TEMP_MSB] = _wire->read();
  _scratchPad[VOLT_LSB] = _wire->read();
  _scratchPad[VOLT_MSB] = _wire->read();
  _scratchPad[CURR_LSB] = _wire->read();
  _scratchPad[CURR_MSB] = _wire->read();
  _scratchPad[THRESH] = _wire->read();
}
