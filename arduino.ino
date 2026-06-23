#include <SPI.h>
#include <MFRC522.h>

// 設定重設腳位 與 SPI 介面裝置選擇腳位
#define RST_Pin 3
#define SS_Pin 2
MFRC522 *mfrc522; // 宣告MFRC522指標

// 馬達腳位設定
int PWMA = 10;
int AIN2 = 6;
int AIN1 = 7;
int BIN2 = 9;
int BIN1 = 8;
int PWMB = 11;
int STBY = 5; // A:左輪，B:右輪

// IR 感測器定義
#define analogPin1 A3
#define analogPin2 A4
#define analogPin3 A5
#define analogPin4 A6
#define analogPin5 A7

auto time = micros();

// --- USER CONFIGURATION ---
#define CUSTOM_NAME "diaob" // Max length is 12 characters
// ---------------------------

long baudRates[] = {9600, 19200, 38400, 57600, 115200, 4800, 2400, 1200, 230400};
bool moduleReady = false;

void setup() {
analogWrite(PWMA, 190);
analogWrite(PWMB, 200);
pinMode(STBY, OUTPUT);
digitalWrite(STBY, HIGH);
pinMode(PWMA, OUTPUT);
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMB, OUTPUT);


pinMode(analogPin1, INPUT);
pinMode(analogPin2, INPUT);
pinMode(analogPin3, INPUT);
pinMode(analogPin4, INPUT);
pinMode(analogPin5, INPUT);

Serial.begin(115200);
while (!Serial);
Serial.println("Initializing HM-10...");

// 1. Automatic Baud Rate Detection
for (int i = 0; i < 9; i++) {
    Serial.print("Testing baud rate: ");
    Serial.println(baudRates[i]);
    Serial3.begin(baudRates[i]);
    Serial3.setTimeout(100);
    delay(100);

    Serial3.print("AT");
    if (waitForResponse("OK", 800)) {
        Serial.println("HM-10 detected and ready.");
        moduleReady = true;
        break;
    } else {
        Serial3.end();
        delay(100);
    }
}

if (!moduleReady) {
    Serial.println("Failed to detect HM-10. Check 3.3V VCC and wiring.");
    return;
}

// 3. Restore Factory Defaults
Serial.println("Restoring factory defaults...");
sendATCommand("AT+RENEW");
delay(500);

// 4. Set Custom Name
Serial.print("Setting name to: ");
Serial.println(CUSTOM_NAME);
String nameCmd = "AT+NAME" + String(CUSTOM_NAME);
sendATCommand(nameCmd.c_str());

// 5. Enable Connection Notifications
Serial.println("Enabling notifications...");
sendATCommand("AT+NOTI1");

// 6. Get the Bluetooth MAC Address
Serial.println("Querying Bluetooth Address");
sendATCommand("AT+ADDR?");

// 7. Restart the module
Serial.println("Restarting module...");
sendATCommand("AT+RESET");
delay(1000);
Serial3.begin(9600);

Serial.println("Initialization Complete.");

// RFID 初始化
SPI.begin();
mfrc522 = new MFRC522(SS_Pin, RST_Pin);
mfrc522->PCD_Init();
Serial3.println(F("Read UID on a MIFARE PICC:"));


}

void MotorWriting(int vL, int vR) {
analogWrite(PWMA, abs(vL));
analogWrite(PWMB, abs(vR));
if (vL > 0) {
digitalWrite(AIN1, LOW);
digitalWrite(AIN2, HIGH);
} else {
digitalWrite(AIN1, HIGH);
digitalWrite(AIN2, LOW);
}
if (vR > 0) {
digitalWrite(BIN1, LOW);
digitalWrite(BIN2, HIGH);
} else {
digitalWrite(BIN1, HIGH);
digitalWrite(BIN2, LOW);
}
}

String dir;
int step = 0;
int prvctrl;
double Kp = 0.5;
double Kd = 2;
double error = 0;
double lastError = 0;
const int Tl = 148*4/3;
const int Tr = 150*4/3;
int testl=0;
int testr=0;
int ctrl = 1;
int state = 1;
int bomega = 80;
int omega=80;
auto turn_start = micros();
auto state_start = micros();
auto loop_start=micros();
int loopcnt=0;

void adjust(int &vL,int &vR,double d){
vL*=d;
vR*=d;
if(vL>vR and vL>255){
double dd=(double)255/vL-0.01;
vL*=dd;
vR*=dd;
}
else if(vR>vL and vR>255){
double dd=(double)255/vR-0.01;
vL*=dd;
vR*=dd;
}
}

void loop() {
// if(loopcnt==100){
//   // Serial3.print("loop_time = ");
//   Serial3.println((double)(micros()-loop_start)/1000000);
//   loopcnt=0;
// }
// loopcnt++;
// loop_start=micros();
if (!mfrc522->PICC_IsNewCardPresent()) {
goto FuncEnd;
}
if (!mfrc522->PICC_ReadCardSerial()) {
goto FuncEnd;
}


for (byte i = 0; i < mfrc522->uid.size; i++) {
    if (mfrc522->uid.uidByte[i] < 0x10) Serial3.print(F("0"));
    Serial3.print(mfrc522->uid.uidByte[i], HEX);
}
Serial3.println();

mfrc522->PICC_HaltA();
mfrc522->PCD_StopCrypto1();


FuncEnd:;
int r3 = 0.75 * analogRead(analogPin1);
int r2 = analogRead(analogPin2);
int m = analogRead(analogPin3);
int l2 = analogRead(analogPin4);
int l3 = analogRead(analogPin5);
error = ((r3 * 2 + r2) - (l3 * 2 + l2));
int derror=error-lastError;
int powerCorrection = Kp * error + Kd * derror;
int vR = Tr - powerCorrection;
int vL = Tl + powerCorrection;
lastError = error;
vR = min(255, max(-255, vR));
vL = min(255, max(-255, vL));


if (ctrl == 0) {
    if (state == 0) {
        if((double)(micros() - state_start) / 1000000 <0.2*0.75){
            MotorWriting(vL,vR);
        }
        else if((double)(micros() - state_start) / 1000000 <0.52){
            adjust(vL,vR,1.25);
            MotorWriting(vL,vR);
        }
        else{
            if((l3>150 and m>150) or (r3>150 and m>150)){
                powerCorrection/=5;
            }
            // powerCorrection=max(min(powerCorrection,10),-10);
            vR = Tr - powerCorrection;
            vL = Tl + powerCorrection;
            vR = min(255, max(-255, vR));
            vL = min(255, max(-255, vL));
            MotorWriting(vL*0.85,vR*0.85);
        }
    } else if (state == 1) {
        MotorWriting(0, 0);
    } else if (state == 11) {
        MotorWriting(Tl*0.6, Tr*0.6);
    } else if (state == 41) { // 又回轉
        if((double)(micros() - turn_start) / 1000000 <0.8/1.45/1.1){
          MotorWriting(bomega*1.35*1.1,-bomega*2.25*1.25*1.1);
        }
        else{
          MotorWriting(bomega/1.5,-bomega/1.5);
        }
    } else if (state == 51) { // 左回轉
        if((double)(micros() - turn_start) / 1000000 <0.8/1.45/1.1){
          MotorWriting(-bomega*2.25*1.25*1.1,bomega*1.3);
        }
        else{
          MotorWriting(-bomega/1.5,bomega/1.5);
        }
    }else if (state == 32) { // 左轉
        if((double)(micros() - turn_start) / 1000000 <0.65/2){
          MotorWriting(0,omega*2);
        }
        else{
          MotorWriting(0,omega/1.2);
        }
    } else if (state==22){ // 右轉
        if((double)(micros() - turn_start) / 1000000 <0.65/1.9){
          MotorWriting(omega*2,0);
        }
        else{
          MotorWriting(omega/1.2,0);
        }
    }

    if (state == 0 && (l2 > 100 && m > 100 && r2 > 100) && (double)(micros() - state_start) / 1000000 >= 0.3) {
        // Serial3.println((double)(micros()-state_start)/1000000);
        state = 1;
        if (step >= (int)dir.length()) {
            MotorWriting(0, 0);
        } else if (step == (int)dir.length() - 1) {
            Serial3.println("nxt");
        }
        ctrl = 1;
    }
    if (state == 11 && (l3+l2+m+r2+r3<1000)) {
        state = 0;
        lastError = 0;
        state_start = micros();
    } else if ((state == 22) && (l2>100 or m>100 or r2>100) && (r3<100) &&(double)(micros() - turn_start) / 1000000 >= 0.4) {
        state = 0;
        lastError = 0;
        state_start = micros();
    } else if ((state == 32) && (r2>100 or m>100 or l2>100) && ( l3<100) && (double)(micros() - turn_start) / 1000000 >= 0.4) {
        state = 0;
        lastError = 0;
        state_start = micros();
    } else if ((state == 41) && (m > 100 || r2 > 100 || r3>100) && (double)(micros() - turn_start) / 1000000 >= 0.65) {
        state = 0;
        lastError = 0;
        state_start = micros();
    }else if ((state == 51) && (m > 100 || l2 > 100 || l3>100) && (double)(micros() - turn_start) / 1000000 >= 0.65) {
        state = 0;
        lastError = 0;
        state_start = micros();
    }
} if(ctrl==1){ // ctrl 1
    if (step >= (int)dir.length()) {
        ctrl = 1;
    } else {
        char response = dir[step];
        if (response == 'f') {
            turn_start = micros();
            state = 11;
        } else if (response == 'r') {
            turn_start = micros();
            state = 22;
        } else if (response == 'l') {
            turn_start = micros();
            state = 32;
        } else if(response=='b') { // response == 'b'
            turn_start = micros();
            state = 41;
        }else{
            turn_start=micros();
            state=51;
        }
        Serial3.println(response);
        ctrl = 0;
        step++;
    }
}
else if (ctrl==2){
  MotorWriting(testl,testr);
}

if (Serial3.available()) {
    String response = Serial3.readStringUntil('\\n');
    bool allflrb = true;
    for (char c : response) {
        if (c != 'f' && c != 'l' && c != 'r' && c != 'b' && c!='B') {
            allflrb = false;
        }
    }
    if(response=="0"){
      prvctrl=ctrl;
      ctrl=2;
    }
    else if(response=="1"){
      ctrl=prvctrl;
    }
    else if ((int)response.length() >= 2 && response.substring(0, 2) == "kp") {
        String str = response.substring(2, (int)response.length() - 2);
        Kp = (double)str.toInt() / 100;
        Serial3.print("Kp : ");
        Serial3.println(Kp);
    } else if ((int)response.length() >= 2 && response.substring(0, 2) == "kd") {
        String str = response.substring(2, (int)response.length() - 2);
        Kd = (double)str.toInt() / 100;
        Serial3.print("Kd : ");
        Serial3.println(Kd);
    }else if ((int)response.length() >= 1 && response.substring(0, 1) == "L") {
        String str = response.substring(1, (int)response.length() - 1);
        testl = str.toInt();
        Serial3.print("testl : ");
        Serial3.println(testl);
    } else if ((int)response.length() >= 1 && response.substring(0, 1) == "R") {
        String str = response.substring(1, (int)response.length() - 1);
        testr = str.toInt();
        Serial3.print("testr : ");
        Serial3.println(testr);
    } else if (allflrb) {
        dir += response;
    } else if (response == "start") {
        Serial3.println("stby");
    }
}


}

void sendATCommand(const char* command) {
Serial3.print(command);
waitForResponse("", 1000);
}

bool waitForResponse(const char* expected, unsigned long timeout) {
unsigned long start = millis();
Serial3.setTimeout(timeout);
String response = Serial3.readString();
if (response.length() > 0) {
Serial.print("HM10 Response: ");
Serial.println(response);
}
return (response.indexOf(expected) != -1);
}