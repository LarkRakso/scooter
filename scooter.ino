#include <ESC.h>
#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0  //Steering
#define RC_CH2_INPUT  A1  //Throttle
#define RC_CH3_INPUT  A2  //Arming
#define RC_CH4_INPUT  A3

//RC variables
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

uint8_t armed = 0;

class vehicleControl {

  struct throttleStruct{
    uint16_t R = 0; 
    uint16_t L = 0;
  };

  //Steering variables
  public: int16_t steering;
  public: int16_t steering_min = 2000;
  public: int16_t steering_max = 0;
  public: int16_t steering_center = 0;

  //Throttle variables
  public: uint16_t throttle_min = 2000;
  public: uint16_t throttle_max = 0;
  public: int16_t throttle_comp;
  public: throttleStruct throttle;
  //public: uint16_t throttle_L;

  public: uint16_t throttle_arm = 900;

  int16_t calculateSteering(uint16_t steeringStalk) {
    //Calculate wanted steering.
    return steering_center - steeringStalk;
  }

  throttleStruct calculateThrottle(uint16_t throttleStalk, int16_t steering) {
    throttleStruct throttle;

    //Calculate wanted output value...
    throttle.R = throttleStalk + steering;
    throttle.L = throttleStalk - steering;
    throttle_comp = 0;

    //...but make sure the output values are not set below min or above max throttle value.
    if (throttle.R > throttle_max) {
      throttle_comp = throttle.R - throttle_max;
      throttle.R = throttle_max;
      throttle.L = throttleStalk - steering - throttle_comp;
    } else if (throttle.L > throttle_max) {
      throttle_comp = throttle.L - throttle_max;
      throttle.L = throttle_max;
      throttle.R = throttleStalk + steering - throttle_comp;
    } else if (throttle.R < throttle_min) {
      throttle_comp = throttle_min - throttle.R;
      throttle.R = throttle_min;
      throttle.L = throttleStalk - steering + throttle_comp;
    } else if (throttle.L < throttle_min) {
      throttle_comp = throttle_min - throttle.L;
      throttle.L = throttle_min;
      throttle.R = throttleStalk + steering + throttle_comp;
    }
  }
};

//Declaring objects
vehicleControl vc;

//ESCs
uint16_t throttle_arm = 900;
ESC esc_R(9);
ESC esc_L(10);

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  calibrateStalks(vc);
  validateCalibrationAndInit(vc);
}

void loop() {
  //Read input values
  rc_read_values();

  //If Ch3 is below a predefined value, continuously send stop commands to the ESCs until the ESCs are armed.
  if (rc_values[RC_CH3] < 1500) {
    while (rc_values[RC_CH3] < 1500) {
      esc_R.stop();
      esc_L.stop();
      Serial.println("UNARMED");
      armed = false;
      delay(500);
      rc_read_values();
    }
  //When an arming value is read from Ch3 and we are not armed, arm the ESCs
  } else if (!armed && (rc_values[RC_CH3] > 1500) && (abs(rc_values[RC_CH2] - vc.throttle_min) < 10) && (abs(vc.steering_center - rc_values[RC_CH1]) < 10)) {
    esc_R.arm();
    esc_L.arm();
    armed = true;
    Serial.println("ARMED");

  //When we are armed: LET's RUN!!!
  } else if (armed) {

    Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
    Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
    Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");

    vc.steering = vc.calculateSteering(rc_values[RC_CH1]);
    Serial.print("Steering:"); Serial.print(vc.steering); Serial.print("\t");
    vc.throttle = vc.calculateThrottle(rc_values[RC_CH2], vc.steering);    
    Serial.print("RIGHT:"); Serial.print(vc.throttle.R); Serial.print("\t");
    Serial.print("LEFT:"); Serial.print(vc.throttle.L); Serial.print("\t");
    Serial.print("COMP:"); Serial.println(vc.throttle_comp);

    //Output the throttle values to the ESCs..
    esc_R.speed(vc.throttle.R);
    esc_L.speed(vc.throttle.L);
    //If we read unvalid values, stop the ESCs
    if ((vc.throttle.R < (vc.throttle_min - 10)) || (vc.throttle.R > (vc.throttle_max + 10)) || (vc.throttle.L < (vc.throttle_min - 10)) || (vc.throttle.L > (vc.throttle_max + 10))) {
      //Stop engines
      esc_R.stop(); 
      esc_L.stop();
      armed = false;
      Serial.println("EMEGENCY STOP 1 - UNARMED");      
    }

    //Just delay the loop a bit...    
    delay(100);

  //Something whent wrong with arming and/or reading value from Ch3 (arming).
  } else {
    esc_R.stop();
    esc_L.stop();
    armed = false;
    Serial.println("EMEGENCY STOP 2 - UNARMED");
  }
}

void calibrateStalks(vehicleControl myVc) {
  uint32_t calibTimer;

  for (int i = 0; i < 4; i++) {
    rc_read_values();
    myVc.steering_center = myVc.steering_center + rc_values[RC_CH1];
    delay(200);
  }
  //Average value of the steering center reading
  myVc.steering_center = myVc.steering_center/4;
  Serial.print("Steering CENTER: ");
  Serial.println(myVc.steering_center);
    
  //Let the stalk calibration continue for 10000 ms (10 s)
  calibTimer = millis();
  while (millis() - calibTimer <= 10000) {
    rc_read_values();
    if (rc_values[RC_CH1] < myVc.steering_min) {
      myVc.steering_min = rc_values[RC_CH1] - 10;
      Serial.print("Steering MIN: ");
      Serial.println(myVc.steering_min);
    }
    if (rc_values[RC_CH1] > myVc.steering_max) {
      myVc.steering_max = rc_values[RC_CH1] + 10;      
      Serial.print("Steering MAX: ");
      Serial.println(myVc.steering_max);
    }
    if (rc_values[RC_CH2] < myVc.throttle_min) {
      myVc.throttle_min = rc_values[RC_CH2] - 10;
      Serial.print("Throttle MIN: ");
      Serial.println(myVc.throttle_min);
    }
    if (rc_values[RC_CH2] > myVc.throttle_max) {
      myVc.throttle_max = rc_values[RC_CH2] + 10;      
      Serial.print("Throttle MAX: ");
      Serial.println(myVc.throttle_max);
    }
    delay(200);
  }
}

bool validateCalibrationAndInit(vehicleControl myVc) {
    //If the input values are within valid ranges, let's initiate the ESCs
  if (myVc.throttle_min < 1100 && myVc.throttle_min > 900 && myVc.throttle_max < 2000 && myVc.throttle_max > 1800 &&
            myVc.steering_min < 1100 && myVc.steering_min > 900 && myVc.steering_max < 2000 && myVc.steering_max > 1800 &&
            myVc.steering_center > 1400 && myVc.steering_center < 1500) {
      esc_R.init(myVc.throttle_min, myVc.throttle_max, myVc.throttle_arm);
      esc_L.init(myVc.throttle_min, myVc.throttle_max, myVc.throttle_arm);
      return true;  
  }
  //...else send stop commands to the ESCs.
  else {
     esc_R.stop();
     esc_L.stop();
     return false;
  }
}

/*
int16_t calculateSteering(uint16_t steeringStalk) {
    //Calculate wanted steering.
    return steering_center - steeringStalk;
}
*/
/*
void calculateThrottle(uint16_t throttleStalk, int16_t steering) {
  //Calculate wanted output value...
  throttle_R = throttleStalk + steering;
  throttle_L = throttleStalk - steering;
  throttle_comp = 0;

  //...but make sure the output values are not set below min or above max throttle value.
  if (throttle_R > throttle_max) {
    throttle_comp = throttle_R - throttle_max;
    throttle_R = throttle_max;
    throttle_L = throttleStalk - steering - throttle_comp;
  } else if (throttle_L > throttle_max) {
    throttle_comp = throttle_L - throttle_max;
    throttle_L = throttle_max;
    throttle_R = throttleStalk + steering - throttle_comp;
  } else if (throttle_R < throttle_min) {
    throttle_comp = throttle_min - throttle_R;
    throttle_R = throttle_min;
    throttle_L = throttleStalk - steering + throttle_comp;
  } else if (throttle_L < throttle_min) {
    throttle_comp = throttle_min - throttle_L;
    throttle_L = throttle_min;
    throttle_R = throttleStalk + steering + throttle_comp;
  }
}
*/