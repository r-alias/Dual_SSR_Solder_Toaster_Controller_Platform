
#define HEAT_ALL_OFF 0
#define HEAT_TOP_ON   1
#define HEAT_BOTTON_ON 2
#define HEAT_ALL_ON  (HEAT_BOTTON_ON | HEAT_TOP_ON)

#define _countof(A) (sizeof(A)/sizeof(A[0]))
#define MINUITS(A) (A*60)
#define HOUR(A)    (A*60*60)

typedef struct _TEMP_PROFILE_DATA {
  int heaterState;
  int targetTemplature;
  int keepTime;
} ProfileState;

ProfileState profile[] = {
  {HEAT_BOTTON_ON , 130, 15},
  {HEAT_ALL_ON    , 190,  0},
  {HEAT_BOTTON_ON , 185, 10},
  {HEAT_ALL_OFF   ,   0,  0}
};

ProfileState profile2[] = {
  {HEAT_ALL_ON    , 130,  15},
  {HEAT_ALL_ON    , 150,  MINUITS(30)},
  {HEAT_ALL_OFF   ,   0,  0},
  {HEAT_ALL_OFF   ,   0,  0}
};

#include <LiquidCrystal.h>
#include <SPI.h>

// LCD(D2-D7)
#define LCDrsPin 2
#define LCDenablePin 3
#define LCDd4Pin 4
#define LCDd5Pin 5
#define LCDd6Pin 6
#define LCDd7Pin 7
// button(D8)
#define StartButton 8
// Beep(D9)
#define TonePin 9
// Tempratier(D10,D12-D13)
#define TemperatureSlavePin 10
#define TemperatureMisoPin 12
#define TemperatureSckPin 13
// PowerControl(A0,A1)
#define Heat1Pin 14
#define Heat2Pin 15

LiquidCrystal lcd(LCDrsPin,LCDenablePin,LCDd4Pin,LCDd5Pin,LCDd6Pin,LCDd7Pin);

#define FRAME_MS 100
#define FPS (1000 / FRAME_MS)

byte state;          // main program mode
ProfileState *targetProfile;
ProfileState currentProfileState;

byte heatState;      // UpDown heater status [b01:Heat1/ON, b10:Heat2/ON, b11:Both Heatn/ON]
byte tableCounter;   // data table counter
int temperatureWait;  // temprature keep time(SEC)
float temperature;    // Temperature

int blinkTimer;      // blink timer
boolean blinkFlag;   // blink ON/OFF flag

void setup() {
  // degug Initialize(SerialMonitor)
  Serial.begin(9600);
  // LCD initialize
  lcd.begin(20, 4);
  // button initialize
  pinMode(StartButton, INPUT_PULLUP);
  // PowerControl initialize
  pinMode(Heat1Pin, OUTPUT);
  pinMode(Heat2Pin, OUTPUT);
  // Temprature initialize
  pinMode(TemperatureSlavePin, OUTPUT);
  digitalWrite(TemperatureSlavePin, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);
  // memory initialize
  state = 0;
}

void loop() {
  tempratureRead();
  switch (state) {
    case 0: // initialize
      lcd.clear();
      currentProfileState.heaterState = 0;
      currentProfileState.targetTemplature = 0;
      tableCounter = 0;
      targetProfile = profile;
      state++;
      break;
    case 1: // start switch wait
      if (digitalRead(StartButton) == LOW) {
        uint16_t count;
        while(digitalRead(StartButton) == LOW) {
          count++;
          if(count > 500) {
            // at hold 500ms, change profile.
            targetProfile = profile2;
            break;
          }
          delay(1);
        }
        tone(TonePin,600,800);  // StartSound
        lcd.clear();
        setTempratureData();
        state++;
      }
      break;
    case 2: // target Temperature
      if (currentProfileState.targetTemplature <= temperature) {
        state++;
      }
      break;
    case 3: // keep time
      if (--temperatureWait <= 0) {
        state++;
      }
      break;
    case 4: // Loop or Finish?
      tableCounter++;
      setTempratureData();
      if (tableCounter < (_countof(profile)-1)) {
        state = 2;
      } else {
        tone(TonePin,600,1500);  // FinishSound
        state++;
      }
      break;
    case 5: // finish switch wait
      if (digitalRead(StartButton) == LOW) {
        state = 0;
      }
      break;
  }
  heatControl();
  lcdDisplay();
  delay(FRAME_MS);
}

void setTempratureData() {
  currentProfileState = targetProfile[tableCounter];
  temperatureWait = currentProfileState.keepTime * FPS;
  heatState = currentProfileState.heaterState;
}

void tempratureRead() {
  unsigned int thermocouple;
  unsigned int internal;
  float disp;
  // read tem
  digitalWrite(TemperatureSlavePin, LOW);
  thermocouple = (unsigned int)SPI.transfer(0x00) << 8;
  thermocouple |= (unsigned int)SPI.transfer(0x00);
  internal = (unsigned int)SPI.transfer(0x00) << 8;
  internal |= (unsigned int)SPI.transfer(0x00);
  digitalWrite(TemperatureSlavePin, HIGH);
  if ((thermocouple & 0x0001) != 0) {
    Serial.print("ERROR: ");
    if ((internal & 0x0004) !=0) {
      Serial.print("Short to Vcc, ");
    }
    if ((internal & 0x0002) !=0) {
      Serial.print("Short to GND, ");
    }
    if ((internal & 0x0001) !=0) {
      Serial.print("Open Circuit, ");
    }    
    Serial.println();
  } else {
    if ((thermocouple & 0x8000) == 0) {
      temperature = (thermocouple >> 2) * 0.25;
    } else {
      temperature = (0x3fff - (thermocouple >> 2) + 1)  * -0.25;
    }
  }
}

void heatControl() {
  if (temperature > currentProfileState.targetTemplature) {
    heatState = 0;
  } else if (temperature < (currentProfileState.targetTemplature - 0.5)) {
    heatState = currentProfileState.heaterState;
  }
  if ((heatState & 1) == 0) {
    digitalWrite(Heat1Pin, LOW);
  } else {
    digitalWrite(Heat1Pin, HIGH);
  }
  if ((heatState & 2) == 0) {
    digitalWrite(Heat2Pin, LOW);
  } else {
    digitalWrite(Heat2Pin, HIGH);
  }
}

void lcdDisplay() {
  lcd.setCursor(3, 0);
  lcd.print("STATUS:");
  switch (state) {
    case 0: // initialize
    case 1: // start switch wait
      lcd.print("-------");
      lcd.setCursor(1, 1);
      if (blinkFlag == true) {
        lcd.print("press START button");
      } else {
        lcd.print("                  ");
      }
      lcd.setCursor(3, 3);
      lcd.print("SWITCH SCIENCE");
      break;
    case 2: // target Temperature
    case 3: // keep time
    case 4: // Loop or Finish?
    case 5: // finish switch wait
      if (state != 5) {
        if (blinkFlag == true) {
          lcd.print("RUNNING");
        } else {
          lcd.print("       ");
        }
      } else {
        lcd.print("FINISH!");
      }
      lcd.setCursor(0, 1);
      if ((heatState & 1) == 0) {
        lcd.print("HEAT1:OFF  ");
      } else {
        lcd.print("HEAT1:ON   ");
      }
      if ((heatState & 2) == 0) {
        lcd.print("HEAT2:OFF");
      } else {
        lcd.print("HEAT2:ON ");
      }
      lcd.setCursor(5, 3);
      lcd.print("WAIT:");
      if (state == 3) {
        lcd.print(temperatureWait / FPS);
        lcd.print(".");
        lcd.print(temperatureWait % FPS * 10 / FPS);
        lcd.print("sec");
      } else {
        lcd.print("---.-  ");
      }
      lcd.print("  ");
      break;
  }
  lcd.setCursor(2, 2);
  if (temperature < 100.0) lcd.print(" ");
  if (temperature < 10.0) lcd.print(" ");
  lcd.print(temperature);
  lcd.print(" / ");
  lcd.print(currentProfileState.targetTemplature);
  lcd.print("  ");
  // blink control
  if (++blinkTimer >= FPS) {
    blinkTimer = 0;
    if (blinkFlag == false) {
      blinkFlag = true;
    } else {
      blinkFlag = false;
    }
  }
}
