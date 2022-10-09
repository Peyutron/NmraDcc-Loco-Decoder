#include <NmraDcc.h>

#define This_Decoder_Address 3
#define numleds 5
byte ledpins [] = {4,5,6,7,8};    //Pines de salida para leds
byte led_direction [] = {0,1,2,0,1};        //0=On/Off, 1=On Forward, 2=On Reverse
boolean led_last_state [] = {false,false,false,false,false};  //Ultimo estado del led
boolean Last_Function_State[] = {false,false,false,false,false};  //These hold the last Fx assignments
uint8_t Decoder_direction = 0;
int tim_delay = 500;
int fade_time = 170;

const int PWM_MAX = 254;
const int left = 10;
const int right = 11;
const int velPwm = 9;
int currentSpeed = 0;
int rateSteps;
int acSpeed;
int locSpeed = 0;
int rateSpeed = 0;
int dirState = 0;
int steps;
int maniobras = 0;
int CV2 = 100;        //Tension de arranque
int CV3 =1;              //Tasa de aceleración
int CV4 = 1;              //Tasa de frenado
int CV5 = 254;        //Tension velocidad Maxima
long previousAcc = 0;
long previousDec = 0;
long intervalAcc = (100*CV3);
long intervalDec = (100*CV4);
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  // The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, This_Decoder_Address},

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, This_Decoder_Address},

  // ONLY uncomment 1 CV_29_CONFIG line below as approprate
  //  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
  //  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps
};

NmraDcc  Dcc ;

uint8_t FactoryDefaultCVIndex = 0;

// Uncomment this line below to force resetting the CVs back to Factory Defaults
// FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

// Uncomment the #define below to print all Speed Packets
#define NOTIFY_DCC_SPEED
#ifdef  NOTIFY_DCC_SPEED
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  Serial.print("notifyDccSpeed: Addr: ");
  Serial.print(Addr, DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  Serial.print(" Speed: ");
  Serial.print(Speed, DEC);
  //  locSpeed = Speed;
  acSpeed = Speed;
  Serial.print(" Steps: ");
  int SpeedStep = (SpeedSteps - 1);
  steps = SpeedStep;
  Serial.print(SpeedStep, DEC);
  Serial.print(" Dir: ");
  Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  dirState = Dir;
  // delay(400);
};
#endif

// Uncomment the #define below to print all Function Packets
#define NOTIFY_DCC_FUNC
#ifdef  NOTIFY_DCC_FUNC
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr, DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp, DEC);

  switch ( FuncGrp )
  {
#ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE
    case FN_0:
      Serial.print(" FN0: ");
      Serial.println((FuncState & FN_BIT_00) ? "1  " : "0  ");
      break;
#endif

    case FN_0_4:
    exec_function( 0, (FuncState & FN_BIT_00)>>4 );
    exec_function( 1, (FuncState & FN_BIT_01));
    exec_function( 2, (FuncState & FN_BIT_02)>>1);
    exec_function( 3, (FuncState & FN_BIT_03)>>2 );
    exec_function( 4, (FuncState & FN_BIT_04)>>3 );
      if (Dcc.getCV(CV_29_CONFIG) & CV29_F0_LOCATION) // Only process Function 0 in this packet if we're not in Speed Step 14 Mode
      {
        Serial.print(" FN 0: ");
        Serial.print((FuncState & FN_BIT_00) ? "1  " : "0  ");
       // delay (100);
      }

      Serial.print(" FN 1-4: ");
      Serial.print((FuncState & FN_BIT_01) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_02) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_03) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_04) ? "1  " : "0  ");
      break;

    case FN_5_8:
      exec_function( 5, (FuncState & FN_BIT_05));
    exec_function( 6, (FuncState & FN_BIT_06)>>1 );
    exec_function( 7, (FuncState & FN_BIT_07)>>2 );
    exec_function( 8, (FuncState & FN_BIT_08)>>3 );
      Serial.print(" FN 5-8: ");
      Serial.print((FuncState & FN_BIT_05) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_06) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_07) ? "1  " : "0  ");
      Serial.println((FuncState & FN_BIT_08) ? "1  " : "0  ");
      break;

    case FN_9_12:
    exec_function( 9, (FuncState & FN_BIT_09));
    exec_function( 10,(FuncState & FN_BIT_10)>>1 );
    exec_function( 11,(FuncState & FN_BIT_11)>>2 );
    exec_function( 12,(FuncState & FN_BIT_12)>>3 );
      Serial.print(" FN 9-12: ");
      Serial.print((FuncState & FN_BIT_09) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_10) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_11) ? "1  " : "0  ");
      Serial.println((FuncState & FN_BIT_12) ? "1  " : "0  ");
      break;

    case FN_13_20:
      exec_function( 13, (FuncState & FN_BIT_13));
      exec_function( 14, (FuncState & FN_BIT_14)>>1 );
      exec_function( 15, (FuncState & FN_BIT_15)>>2 );
      exec_function( 16, (FuncState & FN_BIT_16)>>3 );
      Serial.print(" FN 13-20: ");
      Serial.print((FuncState & FN_BIT_13) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_14) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_15) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_16) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_17) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_18) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_19) ? "1  " : "0  ");
      Serial.println((FuncState & FN_BIT_20) ? "1  " : "0  ");
      break;

    case FN_21_28:
      Serial.print(" FN 21-28: ");
      Serial.print((FuncState & FN_BIT_21) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_22) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_23) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_24) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_25) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_26) ? "1  " : "0  ");
      Serial.print((FuncState & FN_BIT_27) ? "1  " : "0  ");
      Serial.println((FuncState & FN_BIT_28) ? "1  " : "0  ");
      break;
  }
}
void exec_function (int f_index, int FuncState)  {
       if ((FuncState==1) && (!Last_Function_State[f_index])) {
            Last_Function_State[f_index] = true;
      Set_LED (f_index,true);
          }
          else if ((FuncState==0) && Last_Function_State[f_index]) { 
            Last_Function_State[f_index] = false;
      Set_LED (f_index,false);
          }
}
void Set_LED (int Function, boolean led_state) {
boolean start_state = !led_state;
boolean end_state = led_state;
    switch (led_direction[Function]) {
    case 0:                                      //0=On/Off
      if (led_last_state[Function] == led_state) return;
      Switch_LED (Function);
    break;
    case 1:                                     //1=On Forward
    if (Decoder_direction!=0) {
      if (led_last_state[Function] == led_state) return;
      Switch_LED (Function);
    }
    break;
    case 2:                                     //2=On Reverse
    if (Decoder_direction==0)  {
      if (led_last_state[Function] == led_state) return;
      Switch_LED (Function);
    }
    break;
    default:
    break;
  }
}
void Switch_LED (int Function) {
  float time_fraction;
  int del_temp;
  boolean start_state = led_last_state[Function];
  boolean end_state = !led_last_state[Function];
  for (int loop_time=0; loop_time<fade_time; loop_time++)  {
        time_fraction = (float (loop_time))/(float (fade_time));
        digitalWrite (ledpins[Function], start_state);
        del_temp = 1000 - (1000.*time_fraction);
        if (del_temp<0) del_temp=0;
        delayMicroseconds (del_temp);
        digitalWrite (ledpins[Function], end_state);
        delayMicroseconds (1000.*time_fraction);
        }
  led_last_state[Function] = end_state;
}

#endif

// Uncomment the #define below to print all DCC Packets
#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for (uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read

const int DccAckPin = 15 ;

void notifyCVAck(void)
{
  Serial.println("notifyCVAck") ;

  digitalWrite( DccAckPin, HIGH );
  delay( 8 );
  digitalWrite( DccAckPin, LOW );
}

void setup()
{
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(velPwm, OUTPUT);
  pinMode (13,OUTPUT);
  Serial.begin(115200);
  Serial.println("NMRA Dcc Multifunction Decoder Demo 1");

  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );
  digitalWrite( DccAckPin, LOW );
  
    for (int i=0; i< numleds; i++) {
      pinMode(ledpins[i], OUTPUT);
      digitalWrite(ledpins[i], LOW);
     }
  for (int i=0; i< numleds; i++) {
     digitalWrite(ledpins[i], HIGH);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  for (int i=0; i< numleds; i++) {
     digitalWrite(ledpins[i], LOW);
     delay (tim_delay/10);
  }

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, 2, 0);

  // Call the main DCC Init function to enable the DCC Receiver
  //Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY, 0 );

  // Uncomment to force CV Reset to Factory Defaults
  notifyCVResetFactoryDefault();
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if ( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }


  if (dirState == 0) {
    digitalWrite (left, HIGH);
    digitalWrite (right, LOW);

  }
  if (dirState == 1) {
    digitalWrite (right, HIGH);
    digitalWrite (left, LOW);
  }
  if (currentSpeed <= 1) {
    locSpeed = 0;
    digitalWrite (left, LOW);
    digitalWrite (right, LOW);
  }
  else {
    rateSpeed = ((PWM_MAX - CV2) / steps);
  }
  
  if (currentSpeed != acSpeed ) { //si currentSteps es diferente de steps
    if (currentSpeed < acSpeed) {
      Acc();
    }
    if (currentSpeed > acSpeed) {
      Dec();
    }
  }
  Serial.print("Velocidad PWM : ");
  Serial.print(locSpeed);
  analogWrite (velPwm, locSpeed);
}

void Acc() {
  unsigned long currentAccMillis = millis();

  if (currentAccMillis - previousAcc > intervalAcc ) {
    previousAcc = currentAccMillis;
    locSpeed = (CV2 + (currentSpeed * rateSpeed));

    if (locSpeed >= CV5) {  //si velocidad de locomotora mayor que
      locSpeed = CV5;
    }
    currentSpeed = (currentSpeed + 1);
    if (currentSpeed >= acSpeed) {
      currentSpeed = acSpeed;
    }
  }
  return;
}
void Dec() {
  unsigned long currentDecMillis = millis();
  if (currentDecMillis - previousDec > intervalDec) {
    previousDec = currentDecMillis;
    locSpeed = (CV2 + (currentSpeed * rateSpeed));
    currentSpeed = (currentSpeed - 1);
    if (currentSpeed <= acSpeed) {
      currentSpeed = acSpeed;
    }
  }
  return;
}


