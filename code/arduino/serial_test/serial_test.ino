// Message types
const byte MSG_LSM330 = 'L';
const byte MSG_MPU6050 = 'M';
const byte MSG_TIMESTAMP = 'T';

const byte test_msg[] = {'L', 0x0, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x0F,
                         'M', 0x1, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
                         'T' };
                         
const byte data_size = 32;

byte time = 0;

void setup() 
{
  delay(1000);
  
  Serial.begin(115200);

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
