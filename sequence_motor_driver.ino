#define MOTOR_PIN 9
#define MOTOR_MINIMUM_DUTY_CYCLE 37
#define MOTOR_MAXIMUM_DUTY_CYCLE 100

uint8_t gap = MOTOR_MAXIMUM_DUTY_CYCLE - MOTOR_MINIMUM_DUTY_CYCLE;


uint8_t sequence_speed[] = {2, 0, 100, 0, 100, 0, 100};
uint16_t sequence_time[] = {1000.0, 150000.0, 500.0, 150000.0, 500.0, 150000.0, 500.0};


uint8_t len_array = 0;
uint8_t current_speed = 0;

void set_pwm(uint8_t speed){
  current_speed = MOTOR_MINIMUM_DUTY_CYCLE + (speed * (gap/100));
  analogWrite(MOTOR_PIN, current_speed);
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  len_array = sizeof(sequence_speed)/sizeof(uint16_t);

  for (uint8_t i = 0; i < len_array; i++){
    set_pwm(sequence_speed[i]);
    delay(sequence_time[i]);
  }  
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
