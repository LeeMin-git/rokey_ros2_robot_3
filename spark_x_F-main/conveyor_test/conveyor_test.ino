#define RECV_MAX_COUNT  10000

#define PIN_ENA   8
#define PIN_DIR   9
#define PIN_PUL   10


typedef enum _CONVEYOR_STATE {Conveyor_Ready, Conveyor_Run} CONVEYOR_STATE;

unsigned long recv_cnt = 0;

unsigned long time_p = 0;
unsigned long time_serial_p = 0;

unsigned long step_count  = 0;
CONVEYOR_STATE state = Conveyor_Ready;

void setup() {
  Serial.begin(115200);
  Serial.write('s');

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);

  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_DIR, LOW);
}

void step_run(unsigned long time_c) {
  if((time_c - time_p) > 2) { 
    if(state == Conveyor_Run) {

      digitalWrite(PIN_PUL, HIGH);
      delayMicroseconds(1000); // 0.1ms
      digitalWrite(PIN_PUL, LOW);
      delayMicroseconds(1000); // 0.1ms

      time_p = time_c;
    } else if (state == Conveyor_Ready) {
      digitalWrite(PIN_PUL, LOW);
      delayMicroseconds(1000); // 0.1ms
      digitalWrite(PIN_PUL, LOW);
      delayMicroseconds(1000); // 0.1ms
      recv_cnt = 0;
      time_p = time_c;
    }
  }
}

void loop() {
  unsigned long time_c = millis();

  if(Serial.available() > 0) {
    int incommingByte = Serial.read();
    if(incommingByte >='0') recv_cnt = (incommingByte - '0');
    if(recv_cnt == 1) state = Conveyor_Run;
    else if (recv_cnt == 0){ 
      state = Conveyor_Ready;
      recv_cnt = 0;
    }
  }
  if (!Serial){
    state = Conveyor_Ready;
  }
  step_run(time_c);
}
