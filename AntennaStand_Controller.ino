//Lorenzo Gregori

/////////THIS IS A ROUGH DRAFT OF AVIONICS ANTENNA STAND CONTROLLER/////////////////////////////
//function:
//in setup must know position of ground station relative to rocket
//current setup: enter this manually
//future setup: mount gps on the stand to do this automatically
//read packet, calculate angle from stand to rocket, move motors to this angle

//test progress
//motor control works.
//angle calculation function works (but only with example inputs)
//future test
//must test radio reciever function, and if angle calculation from real data is accurate

#include <LoRa.h>
#include <Tic.h>
#include <TinyGPS.h>

#define GPSSerial Serial1

TinyGPS gps;

//#define STEP_ANGLE 1.8
//#define STEPS_PER_REV 400

// The GND, SCL, and SDA pins of the Arduino must each be
// connected to the corresponding pins on each Tic.  You might
// consider connecting the ERR lines of both Tics so that if
// either one experiences an error, both of them will shut down
// until you reset the Arduino.
TicI2C tic1(14);
TicI2C tic2(15);


//input the lat and long dist of antenna stand from the rocket
//int32_t stand_dist_lat = _____________________________;
//int32_t stand_dist_long = ____________________________;

//buffer for packet
char buffer[16];


//struct for packet
typedef struct {
  char header = 0x55;
  float lat;
  char padding1;
  float lon;
  char padding2;
  float altitude;
  char padding3;
} __attribute__((packed)) Packet;

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPSSerial.available())
      gps.encode(GPSSerial.read());
  } while (millis() - start < ms);
}


float lat_zero, long_zero, alt_zero, lat_dat, long_dat;
float theta, theta_prev, phi, phi_prev, theta_now, phi_now, theta_zero, phi_zero, alt_dat;
unsigned long time_now, time_prev;

float flat = 0;
float flon = 0;
unsigned long age = 0;

void setup() {
  //set up serial output
  Serial.begin(9600);
  Wire.begin();

  Serial.println("LoRa Receiver");
  //set up LoRa
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Give the Tic some time to start up.
  delay(20);

  //GPS startup
  GPSSerial.begin(9600);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.
  //tic1 is elevation tic2 is azimuth.
  tic1.energize();
  tic1.haltAndSetPosition(0);
  tic1.setStepMode(TicStepMode::Half);
  tic1.exitSafeStart();
  tic2.energize();
  tic2.haltAndSetPosition(0);
  tic2.setStepMode(TicStepMode::Half);
  tic2.exitSafeStart();


  ///calibrate the stand
  unsigned long calib_time = millis();
  while(millis() - calib_time < 1000){
      parse_packet();
      smartdelay(10);
      gps.f_get_position(&flat, &flon, &age);
  }
  delay(100);
  
  parse_packet();
  float lat_pad = lat_dat;
  float long_pad = long_dat;
  alt_zero = alt_dat;

  //lat_zero = lat_pad - stand_dist_lat;
  //long_zero = long_pad - stand_dist_long;
  lat_zero = flat;
  long_zero = flon;
  calc_angle(lat_pad, long_pad, alt_zero);
  Serial.print("lat_zero: ");
  Serial.print(lat_zero,6);
  Serial.print("    long_zero: ");
  Serial.println(long_zero,6);
  Serial.println(alt_zero);
  Serial.print("lat_pad: ");
  Serial.print(lat_pad);
  Serial.print("    long_pad: ");
  Serial.println(long_pad);

  theta_zero = theta_now; //zero theta
  theta_prev = theta_zero;
  phi_zero = phi_now; //zero phi
  phi_prev = phi_zero;
  time_prev = millis();
  Serial.print("theta_zero: ");
  Serial.print(theta_zero);
  Serial.print("    phi_zero: ");
  Serial.println(phi_zero);
  delay(1000);
}

void loop() {
  //if (LoRa.available()) {

    //parse the radio packet
    parse_packet();
    Serial.print("long: ");
    Serial.print(long_dat,6);
    Serial.print("  lat: ");
    Serial.print(lat_dat,6);
    Serial.print("  alt: ");
    Serial.print(alt_dat);


    //algorithm to get the pos
    calc_angle(lat_dat, long_dat, alt_dat);
    time_now = millis();

    //now do predict calc
    float ratePhi, rateTheta;
    rateTheta = (theta_now - theta_prev) / (time_now - time_prev);
    ratePhi = (phi_now - phi_prev) / (time_now - time_prev);

    theta = theta_now + (rateTheta * 100) - theta_zero;
    phi = phi_now + (ratePhi * 100) - phi_zero;
    //theta = theta_now - theta_zero;
    //phi = phi_now - phi_zero;
    Serial.print("                            theta (horiz): ");
    Serial.print(theta);
    Serial.print("    phi (vert): ");
    Serial.println(phi);

    if(theta > 360 || theta < -360){
      theta_now = theta_prev;
      theta = theta_prev;
    }
    if (phi > 90) {
      phi_now = 90;
      phi = 90;
    }
    if (phi < -20) {
      phi_now = phi_prev;
      phi = phi_prev;
    }

    //control motors
    /////have angle from ground stand to rocket (theta is azimuth --- phi is alt)
    setElevationAndTime(phi, 50);
    setAzimuthAndTime(theta, 50);

    theta_prev = theta_now;
    phi_prev = phi_now;
    time_prev = time_now;
  //}
  //else{
    //setElevationAndTime(90, 50);
  //}
}


void parse_packet() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  //packet size is 16
  if (packetSize == 16) {
    // received a packet
    Serial.println("Received packet ");

    char first = (char)LoRa.read();
    //the header is always 0x55
    if (first == 0x55) {
      buffer[0] = first;
      int i = 1;
      //read into buffer
      while (i < 16) {
        buffer[i] = (char)LoRa.read();
        ++i;
      }
      //parse the buffer into packet
      Packet * packet_ptr = (Packet *) buffer;
      //output lat lon
      //Serial.print("lat: ");
      //Serial.println(packet_ptr->lat);
      //Serial.print("lat_dir: ");
      //Serial.println(packet_ptr->lat_dir);
      //Serial.print("lon: ");
      //Serial.println(packet_ptr->lon);
      //Serial.print("lon_dir: ");
      //Serial.println(packet_ptr->lon_dir);

      lat_dat = (packet_ptr->lat);
      long_dat = (packet_ptr->lon);
      alt_dat = packet_ptr->altitude;
    }
  }
}

void calc_angle(float lat_dat, float long_dat, float alt_dat) {
  float deltaLAT = lat_dat - lat_zero;
  float deltaLON = long_dat - long_zero;
  float deltaALT = alt_dat - alt_zero;

  /*
  ///convert hyp to feet for phi calc
  int R = 20903520;
  float lat1 = lat_zero * (3.1416 / 180);
  float lat2 = lat_dat * (3.1416 / 180);
  float deltaPHI = (lat2 - lat1) * (3.1416 / 180);
  float deltaLAM = (long_dat  - long_zero) * (3.1416 / 180);
  float x = (long_dat - long_zero) * cos((lat1 + lat2) / 2);
  float y = lat2 - lat1;
  float hyp = sqrt(x * x + y * y) * R;
  Serial.print("hyp:  "); 
  Serial.println(hyp);
  */

  float R = 6371;
  float p = 0.017453292519943295;
  float a = 0.5 - cos((lat_dat-lat_zero)*p)/2 + cos(lat_zero*p)*cos(lat_dat*p) * (1-cos((long_dat-long_zero)*p)) / 2;
  float d = 2 * R * asin(sqrt(a));
  float hyp = d * 3280.8;
  Serial.print("hyp:  "); 
  Serial.println(hyp);
  
  //Serial.print("deltaLON: ");
  //Serial.print(deltaLON);
  //Serial.print("  deltaLAT: ");
  //Serial.print(deltaLAT);
  //Serial.print("  deltaALT: ");
  //Serial.println(deltaALT);

  theta_now = atan2(deltaLON, deltaLAT) * (180 / 3.1416);
  phi_now = atan2(deltaALT, hyp) * (180 / 3.1416);
}

void resetCommandTimeout()
{
  tic1.resetCommandTimeout();
  tic2.resetCommandTimeout();
}

void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

void waitForPosition1(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic1.getCurrentPosition() != targetPosition);
}
void waitForPosition2(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic2.getCurrentPosition() != targetPosition);
}

// position in degrees
// We are using half steps
void setElevationAndTime(double pos, uint32_t ms) {
  float pulses = (pos * ((6400 / 360) + 0.5)) * (-2);
  tic2.setTargetPosition(int(pulses));
  waitForPosition2(int(pulses));
}

void setAzimuthAndTime(double pos, uint32_t ms) {
  float pulses = (pos * ((6400 / 360) + 0.5)) * (-3.75);
  tic1.setTargetPosition(int(pulses));
  waitForPosition1(int(pulses));
}
