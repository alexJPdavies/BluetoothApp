# BluetoothApp
Vibration sensor data display, record and analysis

So far I have managed to achive the following:

1) Setiing permissions in AndroidManifrst.xml
2) Create Bluetooth adapter object, then printing list of connected devices to console
3) Creating Variable for MAC address of Vibration sensor, then printing device name to console
4) Create socket between two devices and print to console window
5) Creating a 'do' loop to try multiple times to connect to device if not successful first time

To Do:

1) Establishing iput stream
2) Plotting data in graph
3) Building UI to display graph and values

If time:
1) Build SQLlite db to store values from Bluetooth connection
2) Connect SQLlite dp to graph and display values


Arduino code - Current issue with Printing to BT

// Libaries
#include <SD.h>                 // Enables reading from and writing
#include <SPI.h>                //Enables synchronous serial data protocol communication
#include <Adafruit_Sensor.h>    // Adafruit libary required with hardware
#include <Adafruit_BNO055.h>   //Arduino libary for the MPU6050 sensor by Adafruit
#include <Wire.h>               //Enables communication via I2C
#include "BluetoothSerial.h"    // Enables Bluetooth
#include "arduinoFFT.h"         // Fast Fourier Transform Libaryhttps://github.com/kosme/arduinoFFT

//Define Object
arduinoFFT FFT = arduinoFFT(); // Create FFT object
File myFile; // File
BluetoothSerial SerialBT; //Bluetooth serial
Adafruit_BNO055 myIMU; // Define Accelorometer serial object

//FFT values reuired
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;

// Input and output vector arrays
double vReal[samples];
double vImag[samples];

// Contants defined for print vector function
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// SD card CS pin value required
int pinCS = 3; // Pin D3

void setup()
{
  Serial.begin(115200); // Start serial comunication
  SerialBT.begin("VibrationSensor"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(pinCS, OUTPUT); // Define CS pin, have to do this as CS needs to be low
  
  Serial.println("Adafruit MPU6050  & SD Card Status!");

  // Initialize BNO055
  if (!myIMU.begin()) {
    Serial.println("Failed to find BNO055 chip");
    } else {
        Serial.println("BNO055 Found!");
    }
  // Initialize SD card
  if (SD.begin()) // Using SD.begin function initialsis SD card
  {
    Serial.println("SD card is ready to use."); // if the 'if' statement is true the text will be printed on serial monitor
  } else
  {
    Serial.println("SD card initialization failed"); // if the 'if' statement is not true then the text will be printed on the serial monitor as the 'else' statement catches the false if
    return; // return means the program will be termintaed
  }
  myFile = SD.open("/BNO055.txt", FILE_WRITE);
}

void loop()
{
  // Initialise Bluetooth
    SerialBT.write(Serial.read());
    Serial.write(SerialBT.read());

  // Get new sensor events with readings
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  Serial.println(acc.x());
  SerialBT.println(acc.x());

  // Building new sampled data arrays, only with x axis
  for (int i=0; i<samples; i++) //Reading values & time and storing values in array for the number of samples specified
  {
    imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    vReal[i] = digitalRead (acc.x());// Build data with positive and negative values
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  /* Print the results of the simulated sampling according to time */
  Serial.println("Data:");
  SerialBT.print("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  Serial.println("Weighed data:");
  SerialBT.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  SerialBT.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  SerialBT.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  SerialBT.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x;
  double v;
  FFT.MajorPeak(vReal, samples, samplingFrequency, &x, &v);
  Serial.print(x, 6);
  SerialBT.print(x, 6);
  Serial.print(", ");
  SerialBT.print(", ");
  Serial.println(v, 6);
  SerialBT.println(v, 6);
  //while(1); /* Run Once */
  delay(1); /* Repeat after delay */
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    SerialBT.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
      SerialBT.print("Hz");
      Serial.print(" ");
      SerialBT.print(" ");
      Serial.println(vData[i], 4);
      SerialBT.println(vData[i], 4);
  }
  Serial.println();
  SerialBT.println();

}
