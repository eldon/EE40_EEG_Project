/*
  Microcontroller code for EE40 EEG Project at UC Berkeley.
  Copyright 2013 Eldon Schoop. All rights reserved.
  
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
  
  In this version:
  - Help, and increment/decrement fully functional.
  - Numerical entry seems to be fully functional within margin.
  - Everything should be FULLY OPERATIONAL in this version.
*/

#include <LiquidCrystal.h>
#include <Keypad.h>
#include <DS1803.h>
#include <Wire.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','N'},
  {'C','0','H','E'}
};
byte rowPins[ROWS] = {P1_0, P1_1, P1_2, P1_3}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {P1_4, P1_5, P2_0, P2_1}; //connect to the column pinouts of the keypad
Keypad eeg_keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

LiquidCrystal lcd(P2_2, P2_3, P2_4, P2_5, P2_7, P2_6);

DS1803 pot(0x28);
//state variables for potentiometers
int base_res_offset[2] = {403, 300};
uint8_t potval[2] = {255, 255};
uint8_t last_potval[2];

//state variables for amp gain
float gain[2] = {convRToInstrGain(convStepToR(potval[0], base_res_offset[0])), convRToALPFGain(convStepToR(potval[1], base_res_offset[1]))};

//program state variables
bool selectedPot = 0; // 0 is instr amp, 1 is active LPF
bool lastSelectedPot = 1 - selectedPot;

void setup()
{
  lcd.begin(16, 2);
  lcd.clear();
  dispWelcome();
  delay(40);
}
  
void loop()
{
  char sel_key = eeg_keypad.getKey(); //get keypad input
  if (sel_key){
    if (sel_key == 'U') {
      if (potval[selectedPot] > 0) {
        potval[selectedPot] -= 1; // Up routine
      }
    } else if (sel_key == 'D') {
      if (potval[selectedPot] < 255) {
        potval[selectedPot] += 1; // Down routine
      }
    } else if (sel_key == 'N') {
        selectedPot = 1 - selectedPot; // 2nd routine
    } else if (sel_key == 'E') {
      // Enter routine
    } else if (sel_key == 'H') {
      dispHelp(); // Help routine
    } else if (sel_key == 'C') {
      // Clear routine
    } else {
      numericalGainUpdate(sel_key); // Numeric Key routine
    }
  }
  bool pot_chg_val = updatePotentiometers();
  if (pot_chg_val || lastSelectedPot != selectedPot) {
    lastSelectedPot = selectedPot;
    lcd.clear();
    if (selectedPot == 0) {
      lcd.print("Instr. Amp Gain:");
      lcd.setCursor(0, 1);
      lcd.print(calcGains()[0]);
      lcd.print("X");
    } else if (selectedPot == 1) {
      lcd.print("Active LPF Gain:");
      lcd.setCursor(0, 1);
      lcd.print(calcGains()[1]);
      lcd.print("X");
    }
  }
}

bool updatePotentiometers()
{
  bool pot_change_flag = 0;
  for (int i=0; i<2; i++) {
    if (potval[i] != last_potval[i]) {
      pot_change_flag = 1;
      pot.setPot(255 - potval[i], i); //set pot to inverse since we're wiring from high side
      last_potval[i] = potval[i];
    }
  }
  return pot_change_flag;
}

void numericalGainUpdate(byte initKey)
{
  unsigned int desGain = initKey - '0';
  unsigned int lastDesGain;
  char sel_key;
  while ((sel_key = eeg_keypad.getKey()) != 'E')  {
    if (sel_key == 'C') {
      desGain = 0;
    } else if (sel_key >= '0' && sel_key <= '9') { //hack to check if sel_key is numeric
      sel_key -= '0';
      desGain *= 10;
      desGain += sel_key;
      if (selectedPot == 0 && desGain > 2500) { //error checking for max values
        desGain = 2500;
      } else if (selectedPot == 1 && desGain > 350) {
        desGain = 350;
      }
    }
    if (desGain != lastDesGain) {
      lcd.clear();
      lcd.print("Press enter to");
      lcd.setCursor(0, 1);
      lcd.print("set gain: ");
      lcd.print(desGain);
      lcd.blink();
      lastDesGain = desGain;
    }
  }
  desGain = constrain(desGain, 0, 10000);
  if (selectedPot == 0) {
    potval[selectedPot] = convRToStep(convInstrGainToR(desGain), base_res_offset[selectedPot]);
  } else if (selectedPot == 1) {
    potval[selectedPot] = convRToStep(convALPFGainToR(desGain), base_res_offset[selectedPot]);
  }
  lcd.noBlink();
  lcd.clear();
  lastSelectedPot = 1 - selectedPot; //hack to update LCD and calculate gain.
}

float* calcGains()
{
  gain[0] = convRToInstrGain(convStepToR(potval[0], base_res_offset[0]));
  gain[1] = convRToALPFGain(convStepToR(potval[1], base_res_offset[1]));
  return gain;
}

void dispWelcome()
{
  delay(100);
  lcd.print("Boot complete.");
  lcd.setCursor(0, 1);
  lcd.print("Press any key");
  while (!eeg_keypad.getKey()) {}
  lcd.clear();
}

void dispHelp()
{
  delay(20);
  lcd.clear();
  lcd.print(" Enter gain and");
  lcd.setCursor(0, 1);
  lcd.print("  press Enter.");
  while (!eeg_keypad.getKey()) {}
  lcd.clear();
  lcd.print(" Use arrows to");
  lcd.setCursor(0, 1);
  lcd.print("increment gain.");
  while (!eeg_keypad.getKey()) {}
  lcd.clear();
  lcd.print("2nd key selects");
  lcd.setCursor(0, 1);
  lcd.print("InstrA or A_LPF.");
  while (!eeg_keypad.getKey()) {}
  lcd.clear();
  lastSelectedPot = 1 - selectedPot; //hack to update LCD.
}

float convRToInstrGain(int res_val)
{
  return (float)(10.0*((100000.0 + res_val)/res_val));
}

unsigned int convInstrGainToR(float instr_gain)
{
  return (unsigned int)(1000000/(instr_gain - 10));
}

float convRToALPFGain(unsigned int res_val)
{
  return (float)(1 + (100000.0/res_val));
}

unsigned int convALPFGainToR(float alpf_gain)
{
  return (unsigned int)(float)(100000.0/(alpf_gain - 1));
}

unsigned int convStepToR(uint8_t step_val, int base_res_offset)
{
  return (unsigned int)(float)(base_res_offset + step_val*(10000.0/255.0));
}

uint8_t convRToStep(unsigned int res_val, int base_res_offset)
{
  res_val = constrain(res_val, base_res_offset, 10000 + base_res_offset);
  return (uint8_t)constrain((float)((res_val - base_res_offset)*(255.0/10000.0)), 0, 255);
}

/*
 LCD Pinout:
 =================================
 LCD pin              Connect to
 ---------------------------------
 01 - GND             GND, pot
 02 - VCC             +3V3, pot
 03 - Contrast        Pot wiper
 04 - RS              P2_2
 05 - R/W             GND
 06 - EN              P2_3
 07 - DB0             GND
 08 - DB1             GND
 09 - DB2             GND
 10 - DB3             GND
 11 - DB4             P2_4
 12 - DB5             P2_5
 13 - DB6             P2_7
 14 - DB7             P2_6
 15 - BL+             +3V3
 16 - BL-             GND
 =================================
 
 Keypad Pinout:
 =================================
 Keypad pin           Connect to
 ---------------------------------
 01 - Row0            P1_0
 02 - Row1            P1_1
 03 - Row2            P1_2
 04 - Row3            P1_3
 05 - Col0            P1_4
 06 - Col1            P1_5
 07 - Col2            P2_0
 08 - Col3            P2_1
 09 - Shield          GND
 =================================
*/
