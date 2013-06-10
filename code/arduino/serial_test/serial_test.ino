// Message types
const byte MSG_LSM330 = 'L';
const byte MSG_MPU6050 = 'M';
const byte MSG_TIMESTAMP = 'T';

const byte test_msg[] = {'L', 0x0, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x0F,
                         'M', 0x1, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                         'T' };
                         
const byte data_size = 32;

byte time = 0;

// SPI chip select lines for the LSM330s
const byte NUM_LSM330 = 3;
const byte cs_g_pin[3] = {14, 16, 2};
const byte cs_a_pin[3] = {15, 17, 3};

void setup() 
{
  delay(1000);
  
  Serial.begin(115200);
  
  for (byte i = 0; i < NUM_LSM330; i++) {
    pinMode(cs_g_pin[i], OUTPUT);
    pinMode(cs_a_pin[i], OUTPUT);
    digitalWrite(cs_g_pin[i], HIGH);
    digitalWrite(cs_a_pin[i], HIGH);
  }

  delay(10);
  
} // setup()

void loop() 
{ 
  Serial.write(test_msg, data_size);
  Serial.write((byte)0x0);
  Serial.write((byte)0x0);
  Serial.write((byte)0x0);
  Serial.write(time++);
  Serial.write((byte)0x0);
  delay(1000);
  
} // loop()
