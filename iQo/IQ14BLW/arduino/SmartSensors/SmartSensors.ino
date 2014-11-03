#include "DHT.h"

#define DHTPIN 0     // what pin we're connected to

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define I2CSPEED 4000

int scl[2] = {19, 4};
int sda[2] = {18, 5};
byte cmd_buffer[10] = {0};
int cmd_buffer_idx = 0;
bool cmd_true = false;
bool started = false; // global data
DHT dht(DHTPIN, DHTTYPE);
int bt_mcu_notify = 0;

void I2C_delay() 
{ 
  volatile int v; 
  int i; 
  for (i=0; i < I2CSPEED/2; i++) 
    v; 
}

bool read_SCL(char i2c_idx) // Set SCL as input and return current level of line, 0 or 1
{
  pinMode(scl[i2c_idx], INPUT);
  if (digitalRead(scl[i2c_idx]) == HIGH) {
    return true;
  } else {
    return false;
  }
}

bool read_SDA(char i2c_idx) // Set SDA as input and return current level of line, 0 or 1
{
  pinMode(sda[i2c_idx], INPUT);
  if (digitalRead(sda[i2c_idx]) == HIGH) {
    return true;
  } else {
    return false;
  }  
}

void clear_SCL(char i2c_idx) // Actively drive SCL signal low
{
  pinMode(scl[i2c_idx], OUTPUT);
  digitalWrite(scl[i2c_idx], LOW);
}
void clear_SDA(char i2c_idx) // Actively drive SDA signal low
{
  pinMode(sda[i2c_idx], OUTPUT);
  digitalWrite(sda[i2c_idx], LOW);  
}

void arbitration_lost(void)
{
  while (1) {
  }  
}
 
void i2c_start_cond(char i2c_idx) {
  if (started) { // if started, do a restart cond
    // set SDA to 1
    read_SDA(i2c_idx);
    I2C_delay();
    while (read_SCL(i2c_idx) == 0) {  // Clock stretching
      // You should add timeout to this loop
    }
    // Repeated start setup time, minimum 4.7us
    I2C_delay();
  }
  if (read_SDA(i2c_idx) == 0) {
    arbitration_lost();
  }
  // SCL is high, set SDA from 1 to 0.
  clear_SDA(i2c_idx);
  I2C_delay();
  clear_SCL(i2c_idx);
  started = true;
}
 
void i2c_stop_cond(char i2c_idx){
  // set SDA to 0
  clear_SDA(i2c_idx);
  I2C_delay();
  // Clock stretching
  while (read_SCL(i2c_idx) == 0) {
    // add timeout to this loop.
  }
  // Stop bit setup time, minimum 4us
  I2C_delay();
  // SCL is high, set SDA from 0 to 1
  if (read_SDA(i2c_idx) == 0) {
    arbitration_lost();
  }
  I2C_delay();
  started = false;
}
 
// Write a bit to I2C bus
void i2c_write_bit(char i2c_idx, bool b) {
  if (b) {
    read_SDA(i2c_idx);
  } else {
    clear_SDA(i2c_idx);
  }
  I2C_delay();
  while (read_SCL(i2c_idx) == 0) { // Clock stretching
    // You should add timeout to this loop
  }
  // SCL is high, now data is valid
  // If SDA is high, check that nobody else is driving SDA
  if (b && read_SDA(i2c_idx) == 0) {
    arbitration_lost();
  }
  I2C_delay();
  clear_SCL(i2c_idx);
}
 
// Read a bit from I2C bus
bool i2c_read_bit(char i2c_idx) {
  bool b;
  // Let the slave drive data
  read_SDA(i2c_idx);
  I2C_delay();
  while (read_SCL(i2c_idx) == 0) { // Clock stretching
    // You should add timeout to this loop
  }
  // SCL is high, now data is valid
  b = read_SDA(i2c_idx);
  I2C_delay();
  clear_SCL(i2c_idx);
  return b;
}
 
// Write a byte to I2C bus. Return 0 if ack by the slave.
bool i2c_write_byte(char i2c_idx,
                    bool send_start,
                    bool send_stop,
                    unsigned char B) {
  unsigned b;
  bool nack;
  if (send_start) {
    i2c_start_cond(i2c_idx);
  }
  for (b = 0; b < 8; b++) {
    i2c_write_bit(i2c_idx, (B & 0x80) != 0);
    B <<= 1;
  }
  nack = i2c_read_bit(i2c_idx);
  if (send_stop) {
    i2c_stop_cond(i2c_idx);
  }
  return nack;
}
 
// Read a byte from I2C bus
unsigned char i2c_read_byte(char i2c_idx, bool nack, bool send_stop) {
  unsigned char B = 0;
  unsigned b;
  for (b = 0; b < 8; b++) {
    B = (B << 1) | i2c_read_bit(i2c_idx);
  }
  i2c_write_bit(i2c_idx, nack);
  if (send_stop) {
    i2c_stop_cond(i2c_idx);
  }
  return B;
}

unsigned char get_half_oct(unsigned char B)
{
  if (B >= '0' && B <= '9') {
    return (B - '0');
  } else if (B >= 'A' && B <= 'F') {
    return (B - 'A' + 10);
  } else if (B >= 'a' && B <= 'f') {
    return (B - 'a' + 10);
  } else {
    return 0;
  }
}

void bt_mcu_inpt_handler(void)
{
  bt_mcu_notify = 1;
}

void setup() {
  // put your setup code here, to run once
  dht.begin();
  pinMode(scl[0], INPUT);
  pinMode(scl[1], INPUT);
  pinMode(sda[0], INPUT);
  pinMode(sda[1], INPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
  Serial.print("Please input cmd: ");
  
  attachInterrupt(0, bt_mcu_inpt_handler, CHANGE);
}

void loop() {
  char incoming = 0;
  unsigned char reg_addr = 0;
  unsigned char reg_value = 0;
  
  // put your main code here, to run repeatedly:
  if (bt_mcu_notify == 1) {
    bt_mcu_notify = 0;
    Serial.println("Got interrupt from BT");
  }
  
  if (Serial.available() > 0) {
    incoming = Serial.read();
    if (incoming == 0x0D) {
      cmd_true = true;
      Serial.print("\n");
    } else {
      cmd_buffer[cmd_buffer_idx++] = incoming;
      Serial.print(incoming);  
    }
  }
  
  if (cmd_true == true) {
    cmd_true = false;
    cmd_buffer_idx = 0;
    if (cmd_buffer[0] != '0' && cmd_buffer[0] != '1') {
      Serial.println("Unknown I2C target!");
      Serial.print("Please input cmd: ");
      return;  
    }    
    if (cmd_buffer[2] == '0') {
      // Read
      reg_addr = cmd_buffer[4] - '0';
      if (reg_addr > 7) {
        Serial.println("Invalid address!");
        Serial.print("Please input cmd: ");        
      } else {
        i2c_write_byte((cmd_buffer[0] - '0'), true, false, 0x88);
        delay(1);
        i2c_write_byte((cmd_buffer[0] - '0'), false, true, reg_addr);
        delay(1);
        i2c_write_byte((cmd_buffer[0] - '0'), true, false, 0x89);
        delay(1);
        Serial.println(i2c_read_byte((cmd_buffer[0] - '0'), true, true), HEX);
        Serial.print("Please input cmd: ");        
      }
    } else if (cmd_buffer[2] == '1') {
      // Write
      reg_addr = cmd_buffer[4] - '0';
      if (reg_addr > 7) {
        Serial.println("Invalid address!");
        Serial.print("Please input cmd: ");        
      } else {
        reg_value = (get_half_oct(cmd_buffer[6]) << 4) | get_half_oct(cmd_buffer[7]);
        i2c_write_byte((cmd_buffer[0] - '0'), true, false, 0x88);
        delay(1);
        i2c_write_byte((cmd_buffer[0] - '0'), false, false, reg_addr);
        delay(1);
        i2c_write_byte((cmd_buffer[0] - '0'), false, true, reg_value);
        Serial.println("Write done");
        Serial.print("Please input cmd: ");  
      }        
    } else {
      Serial.println("Unknown command!");
      Serial.print("Please input cmd: ");
    }
  }
  
  //int sensorValue = analogRead(A0);//use A0 to read the electrical signal
  //Serial.print("sensorValue ");
  //Serial.println(sensorValue);
  /*
  if (dht.doRead() == false) {
    return;
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (!(isnan(t) || isnan(h))) {
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.println(" *C");
  }
  */
}

