#include "ICM_20948.h"          // Import IMU library
ICM_20948_I2C myICM;            // Creates IMU as imported object

const uint8_t sample = 150;     // Control how many samples are taken as the key--NOTE: this will severely impacy the memory of device.
uint16_t T = 2500;              // How long the recording of "key" is; in miliseconds
uint8_t dT = T/sample;          // Time interval between each sample; Total time / number of samples

float tol = 0.5;          // Tolerance value as percentage of the detected value
float perAccept = 0.65;   // Percent of all values that has to be within tolerances for lock to unlock
int toleOffset = 25;      // Tolerance offset--used to give some leeway to small values and delayed action

int keyAccX[sample];    // Array of recorded acceleration in the X direction
int keyAccY[sample];    // Array of recorded acceleration in the Y direction
int keyAccZ[sample];    // Array of recorded acceleration in the Z direction

bool isLocked = false;  // Indicates whether the 'device' is locked or not

String userInput;       // User entered string/input

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);   // Set up LED pin as OUTPUT; allow LED to be turn on/off

  Serial.begin(250000);           // Set up serial communications
  while(!Serial){};               // Loop to make sure Serial communication is set up before progressing

  Wire.begin();                   // Set up I2C communication
  Wire.setClock(400000);          // Set the I2C clock rate

  // Set up communication between the IMU and Featherboard--will not progress until communication is established
  bool initialized = false;
  while( !initialized ){

    myICM.begin(Wire, 1);

    if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "Trying again..." );
      delay(500);
    }
    else initialized = true;
  }

  // Indicate that device is ready to user. Explains the input commands to user.
  Serial.println("Ready.");
  Serial.println("Please enter \"Lock\" to lock, \"Unlock\" to unlock, and \"Key\" to show recorded key values.");
  Serial.println("The LED will light up when device is locked. LED will be off if device is unlocked.");
}

void loop() {

  //  If a message is sent from the PC to the Featherboard, read the message and take appropriate actions
  //  If user enter: Lock--Device will initiate recording functions and 'lock' device
  //  If user enter: Unlock--Device will initiate unlocking functions to enter motion and compare to key
  //  If user enter: Key--Device will display the recorded acceleration values
  if(Serial.available() > 0){

    userInput = Serial.readString();            // Read user input
    Serial.print("\n User entered: ");
    Serial.println(userInput);                  // Display what user entered

    if(userInput == "Lock\n") recordKey();      // Initiate recordKey functions if 'Lock' is entered

    else if(userInput == "Unlock\n" ) unlock(); // Initiate unlock functions if 'Unlock' is entered

    else if(userInput == "Key\n" ) printKey();  // Initiate printKey functions if 'Key' is entered
  }

  checkLocked();        // Check whether lock is on or off; Turn on LED if locked, Turn off LED if unlocked

}

/* Function to record the acceleration readings from the IMU and save the values as 'Key' */
void recordKey(){

  // If device is not locked, initiate recording procedure to record acceleration values
  if(!isLocked){

    //  Display count down to user so user is prepared for recording
    Serial.println("Record key ...");
    delay(1500);
    Serial.println("In 3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Recording...");
    delay(250);

    //  Save the acceleration values in 3 directions in integer arrays
    //  Save N number of samples (based on global variable) for T amount of seconds
    for( uint8_t i = 0; i < sample; i++ ){
        myICM.getAGMT();
        keyAccX[i] = myICM.accX();
        keyAccY[i] = myICM.accY();
        keyAccZ[i] = myICM.accZ();
        Serial.println(".");
        delay(dT);
    }

    // Indicate to user that recording is finished and enter 'locked' condition
    Serial.println("End of recording.");
    isLocked = true;
  }

  // If device is locked, tell the user that device is already locked
  else Serial.println("Virtual locking mechanism is already locked.");
}

/* Function to check whether the device is locked and turn LED on/off accordingly */
void checkLocked(){
  if(isLocked) digitalWrite(LED_BUILTIN, HIGH);   // Turn ON LED if device is locked
  else digitalWrite(LED_BUILTIN, LOW);            // Turn OFF LED if device is unlocked
}

/* Function to unlock the device by measuring acceleration values of gesture and compare to the key recording */
void unlock(){

  // If device is locked, prompt user to enter "Key" to compare reading to recorded values
  if(isLocked){

    //  Display countdown to let user know unlocking sequence is starting
    Serial.println("Begin unlocking sequence...");
    delay(1500);
    Serial.println("In 3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Enter Key...");
    delay(250);

    // Variables to keep track of number of accepted readings and minimum amount of readings within tolerated range
    int numCorrect = 0;
    int minimumCorrect = 3* sample * perAccept;   // Minimum number of correct reading to consider the motion to be key

    // Tolerances of the reading comparison
    int tolX;
    int tolY;
    int tolZ;

    //    Read the acceleration values of the motion and compare if they are within range of tolerance
    //    The device will be unlocked if at least X% (as defined) of the readings are within range of tolerance
    for( uint8_t i = 0; i < sample; i++ ){
        myICM.getAGMT();

        //  Tolerances of the recorded acceleration values (tol% of the value plus offset)
        //  Tolerance values is calculated from percentage of the actual key value plus some defined offset
        //  Offset is used to define minimum value of tolerance if the reading is small and
        //  to account for the delayed actions
        tolX = abs(keyAccX[i]*tol) + toleOffset;
        tolY = abs(keyAccY[i]*tol) + toleOffset;
        tolZ = abs(keyAccZ[i]*tol) + toleOffset;

        //    Compare the acceleration value of motion to recorded key values; Count as correct if value is within tolerance
        if ( myICM.accX()<=keyAccX[i]+tolX && myICM.accX()>=keyAccX[i]-tolX) numCorrect++;
        if ( myICM.accY()<=keyAccY[i]+tolY && myICM.accY()>=keyAccY[i]-tolY) numCorrect++;
        if ( myICM.accZ()<=keyAccZ[i]+tolZ && myICM.accZ()>=keyAccZ[i]-tolZ) numCorrect++;

        Serial.println(".");
        delay(dT);
    }

    // Indicate to user that sequence has ended and enter lock/unlock status
    Serial.println("End of sequence. \n");

    if(numCorrect >= minimumCorrect ) isLocked = false;
    else isLocked = true;

  }

  // If device is unlocked, indicate to user that device is already unlocked
  else Serial.println("Virtual locking mechanism is already unlocked.");
}

/* Function to print out the recorded acceleration values (Key) */
void printKey(){
  for( uint8_t i = 0; i < sample; i++ ){
      Serial.print("[ ");
      Serial.print(keyAccX[i]);
      Serial.print(", ");
      Serial.print(keyAccY[i]);
      Serial.print(", ");
      Serial.print(keyAccZ[i]);
      Serial.println(" ]");
  }
}
