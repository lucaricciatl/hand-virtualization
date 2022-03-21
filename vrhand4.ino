#include <Wire.h>

#define MPU_NUMBER      3
#define BAUD_RATE       115200  // 9600
#define TCA_ADDR        0x70
#define MPU_ADDR        0x68
#define CHECK_6050      0x77
#define AK8963_ADDR     0xC     // AK8963_ADDR (0x0C)
#define AK8963_CNTL     0xA
#define PWR_MGMT_1      0x6B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_FS_2      0x00    // ± 2g
#define GYRO_FS_250     0x00    // ± 250 °/s 
#define START_READ_DATA 0x3B    // ACCEL_XOUT_H
#define N_REG_ACC_GYRO  14
#define TCA_DELAY       10      // waits for mux switch to stabilize
#define N_TCA_I2C       8
#define I2C_FREQUENCY   400000  // set to 100000 if some problems appears
#define INT_PIN_CFG     0x37
#define AK8963_ENABLE   0x02    // ± 4800 uT (not programmable)
#define AK8963_DELAY    10
#define READ_DATA_MAGN  0x03    // start read on AK8963_XOUT_L (little-endian)
#define READ_ASA_MAGN   0x10    // start read on AK8963_ASAX: for magnetometer sensitivity adjustment
#define MAGN_MODE       0x06    // 100 Hz continuous magnetometer data read
#define _3DOF           3
#define N_REG_MAGN_OF   7       // magn reg + magnetic sensor overflow register AK8963_ST2 (0x09)
#define LOOP_DELAY      1       // delay between each data read update

bool  calibrate_gyro =  false;
bool  calibrate_magn =  true;             // if true, set calibrate_gyro to false
bool  console =         false;            // when true prints some additional info
float corr_acc =        16384.0;          // LSB/g
float corr_gyro =       131.0;            // LSB/(º/s)
float conv_magn =       10.0;             // conversion: 1.0 = uT ; 10.0 = mG
float corr_magn =       4912.0 / 32760.0; // uT/LSB

using std::vector;
vector<int16_t>   raw_data(6);            // acc+gyro
vector<int16_t>   raw_data_magn(3);       // acc+gyro
vector<float>     data(6);                // acc+gyro
vector<float>     data_magn(3);           // acc+gyro
vector<float>     gyro_offset(3);         // for gyro calibration
vector<uint8_t>   mpu_on_tca_channel = {2,3,4}; // location of MPUs on mux channels
vector<float>     magn_bias_factory(3);
vector<float>     magn_bias(3);
vector<float>     magn_scale(3);


void setup() {
  delay(1000);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY);
  Serial.begin(BAUD_RATE);
  for (uint8_t i = 0 ; i < MPU_NUMBER ; i++){
    tcaselect(mpu_on_tca_channel[i]);
    delay(TCA_DELAY);
    setup_imu();
  }
  //I2C_scan();
}

void loop() {
  for (uint8_t i = 0 ; i < MPU_NUMBER ; i++){
    tcaselect(mpu_on_tca_channel[i]);
    delay(TCA_DELAY);
    read_data();
    if (console) Serial.print("mpu ");
    Serial.print(i);Serial.print("\t");
    print_data();
  }
  delay(LOOP_DELAY);
}


void read_raw_data(){
  readWire(MPU_ADDR, START_READ_DATA, N_REG_ACC_GYRO);
  for (uint8_t i = 0 ; i < raw_data.size()+1 ; i++){
    if (i == 3){ // skip Temperature reading
      Wire.read();Wire.read();
    }
    else if (i < 3)
      raw_data[i] = Wire.read()<<8|Wire.read();
    else
      raw_data[i-1] = Wire.read()<<8|Wire.read();
  }

  readWire(AK8963_ADDR, READ_DATA_MAGN, N_REG_MAGN_OF);
  //if (!(magn_reg[N_REG_MAGN_OF-1] & MAGN_OVFL_CHECK))
  for (uint8_t i = 0 ; i < _3DOF ; i++){
    int8_t lowByte = Wire.read();
    raw_data_magn[i] = Wire.read()<<8|lowByte; // (little-endian)
  }
}

void read_data(){
  read_raw_data();
  for (uint8_t i = 0 ; i < data.size() ; i++){
    if (i < 3)  // accelerometer data
      data[i] = (float)raw_data[i] / corr_acc;
    else        // gyroscope data
      data[i] = ( (float)raw_data[i] / corr_gyro ) - gyro_offset[i-3];
  }
  float magn_resolution = corr_magn * conv_magn;
  for (uint8_t i = 0 ; i < data.size() ; i++){
    data_magn[i] = (float)(raw_data_magn[i] * magn_resolution * magn_bias_factory[i] - magn_bias[i]) * magn_scale[i];
  }
}

void print_data(){
  for (uint8_t i = 0 ; i < data.size() ; i++){
    Serial.print(data[i]);
    if (i != data.size() - 1)
      Serial.print("\t");
  }
  for (uint8_t i = 0 ; i < _3DOF ; i++){
    // read magnetometer AK8963
    Serial.print("\t");Serial.print(data_magn[i]);
  }
  Serial.println();
}

void setup_imu(){
  writeWire(MPU_ADDR, ACCEL_CONFIG, ACCEL_FS_2);
  writeWire(MPU_ADDR, GYRO_CONFIG, GYRO_FS_250);
  writeWire(MPU_ADDR, PWR_MGMT_1, 0); // set to zero (wakes up the MPU)
  // configure the magnetometer for continuous read and highest resolution (100 Hz sample rates)
  uint8_t magn_resp = initAK8963(); // if the MPU doesn't have an AK8963 magnetometer (ex. MPU-6050), this command does nothing
  delay(10);
  if (calibrate_gyro) calc_gyro_offset();
  if (calibrate_magn && !magn_resp) calc_magn_hard_iron_soft_iron_offset();
}

uint8_t initAK8963(){
  writeWire(MPU_ADDR, INT_PIN_CFG, AK8963_ENABLE); delay(AK8963_DELAY);
  uint8_t r = writeWire(AK8963_ADDR, AK8963_CNTL, 0xF); delay(AK8963_DELAY); // enter Fuse ROM access mode
  if (r == 0) { // the magnetometer responded
    readWire(AK8963_ADDR, READ_ASA_MAGN, _3DOF);
    for (uint8_t i = 0 ; i < _3DOF ; i++)
      magn_bias_factory[i] = (float)(Wire.read() - 128) / 256.0 + 1.0;
    // Configure the magnetometer for continuous read (100 Hz sample rates) and highest resolution (16 bit)
    uint8_t r = writeWire(AK8963_ADDR, AK8963_CNTL, 1 << 4 | MAGN_MODE); delay(AK8963_DELAY); // 0x16
    if (console) Serial.println(r);
  }
  return r;
}

void calc_magn_hard_iron_soft_iron_offset() {
  if(console){
    Serial.println("========================================");
    Serial.println("Calculating magn offset");
    Serial.print("WAVE DEVICE IN A FIGURE EIGHT UNTIL DONE");}
  uint16_t n_test = 1500; // new mag data is available every 10 ms --> ~15 seconds of magn data
  int32_t average_bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767};
  int16_t mag_min[3] = {32767, 32767, 32767};
  for (uint16_t i = 0; i < n_test; i++) {
    read_raw_data(); // update
    for (uint8_t j = 0; j < _3DOF; j++) {
      if (raw_data_magn[j] > mag_max[j]) mag_max[j] = raw_data_magn[j];
      if (raw_data_magn[j] < mag_min[j]) mag_min[j] = raw_data_magn[j];
    }
    delay(12); // wait for next available data
  }
  // Get hard iron correction
  average_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
  average_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
  average_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts
  float bias_resolution = conv_magn * corr_magn;
  magn_bias[0] = (float)average_bias[0] * bias_resolution * magn_bias_factory[0]; // save mag biases in G for main program
  magn_bias[1] = (float)average_bias[1] * bias_resolution * magn_bias_factory[1];
  magn_bias[2] = (float)average_bias[2] * bias_resolution * magn_bias_factory[2];
  // Get soft iron correction estimate
  scale[0] = (float)(mag_max[0] - mag_min[0]) * magn_bias_factory[0] / 2; // get average x axis max chord length in counts
  scale[1] = (float)(mag_max[1] - mag_min[1]) * magn_bias_factory[1] / 2; // get average y axis max chord length in counts
  scale[2] = (float)(mag_max[2] - mag_min[2]) * magn_bias_factory[2] / 2; // get average z axis max chord length in counts
  float avg_rad = scale[0] + scale[1] + scale[2];
  avg_rad /= 3.0;
  magn_scale[0] = avg_rad / ((float)scale[0]);
  magn_scale[1] = avg_rad / ((float)scale[1]);
  magn_scale[2] = avg_rad / ((float)scale[2]);
  if (console) {
    Serial.println("Done!");
    Serial.println("========================================");}
}

void calc_gyro_offset(){
  int n_test = 3000;
  float gx = 0, gy = 0, gz = 0;
  if(console){
    Serial.println("========================================");
    Serial.println("Calculating gyro offset");
    Serial.print("DO NOT MOVE MPU");}
  for(int i = 0; i < n_test; i++){
    read_data(); // update
    gx += data[3]; // gyroX
    gy += data[4]; // gyroY
    gz += data[5]; // gyroZ
  }
  gyro_offset[0] = gx / n_test;
  gyro_offset[1] = gy / n_test;
  gyro_offset[2] = gz / n_test;
  if(console){
    Serial.println("Done!");
    Serial.println("========================================");}
}

int I2C_scan() { // checks I2C channel for devices that may answer on any address
  if(console){
    Serial.println("             I2C SCANNING            ");
    Serial.println();
    Serial.println("-------------------------------------");
    Serial.println();}
  int devices = 0, flag = 0;
  for (uint8_t t = 0; t < N_TCA_I2C ; t++) {
    tcaselect(t);
    flag = 0;
    if(console){Serial.print("TCA Port #");Serial.print(t);Serial.print("\t");}
    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCA_ADDR) continue; // Don't report on the TCA9548A itself!
      Wire.beginTransmission(addr);
      int response = Wire.endTransmission(); // see if something acknowledged the transmission
      if (response == 0) {
        if(console){Serial.print("Found I2C 0x");Serial.println(addr, HEX);}
        flag = 1;
        devices = devices + 1;
      }
    }
    if (flag == 0) {if(console) {Serial.print("Device not found");Serial.println();}}
    delay(100); // Slow the loop scanner down a bit
  }
  if(console){
    Serial.println();
    Serial.println("\nScan completed.");
    Serial.println();
    Serial.print(devices);Serial.println(" device found ");}
  return devices;
}

//========================================//
//================= APIs =================//
//========================================//

uint8_t tcaselect(uint8_t i){
  if (i > 7) return 5; // endTransmission can return a number between 0 and 4
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  uint8_t response = Wire.endTransmission(true);
  return response;
}

uint8_t writeWire(uint8_t addr, uint8_t reg, uint8_t data){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  uint8_t response = Wire.endTransmission(true);
  return response;
}

uint8_t readWire(uint8_t addr, uint8_t start_reg, uint8_t n_reg){
  Wire.beginTransmission(addr);
  Wire.write(start_reg);
  uint8_t response = Wire.endTransmission(false);
  Wire.requestFrom(addr, n_reg); // n_reg is the number of registers requested
  return response;
}

uint8_t checkWire(uint8_t addr){
  Wire.beginTransmission(addr);
  uint8_t response = Wire.endTransmission(true);
  return response;
}
