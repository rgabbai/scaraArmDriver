


void initMotorDriver() {
    // Z-Axis Step motor   
        
    // Global enable 
    pinMode(enable,OUTPUT);
    //X-Stepper
    pinMode(XstepPin,OUTPUT); 
    pinMode(XdirPin,OUTPUT);
    //Y-Stepper
    pinMode(YstepPin,OUTPUT); 
    pinMode(YdirPin,OUTPUT);
    //Z-Stepper
    pinMode(ZstepPin,OUTPUT); 
    pinMode(ZdirPin,OUTPUT);

    // Limit switch 
    pinMode(XLimitPin, INPUT_PULLUP);
    pinMode(YLimitPin, INPUT_PULLUP);
    pinMode(ZLimitPin, INPUT_PULLUP);
    
    //JOY pins
    pinMode(vrx , INPUT); 
    pinMode(vry , INPUT); 
    pinMode(sw  , INPUT);


    digitalWrite(enable,HIGH);
    digitalWrite(XdirPin,HIGH); 
    digitalWrite(YdirPin,HIGH); 
    digitalWrite(ZdirPin,HIGH); 
    digitalWrite(XstepPin,HIGH);
    digitalWrite(YstepPin,HIGH); 
    digitalWrite(ZstepPin,HIGH); 



    Serial.println("Arm Initialization done");


}

bool isButtonPressed() {
//detect valid sw press
// Read the state of the switch into a local variable
  int reading = analogRead(sw);
  if (reading == 0) {
    buttonState+=1;
  }
  if (buttonState > buttonStateCount){
    buttonState = 0;
    return true;
  }
  return false;
}
void resetStepMotor(char smNum) {
// Reset motor location using limit micro switch
Serial.print("Request reset:");
Serial.println(smNum);


    switch(smNum) {

    case SM_X:
        Serial.println("Start X location setup");
        digitalWrite(enable,LOW);
        digitalWrite(XdirPin,HIGH); // Go Right
        while (Xlimit)
        {
            digitalWrite(XstepPin,HIGH); 

            delayMicroseconds(XSM_RST_SPEED); 
            digitalWrite(XstepPin,LOW); 
            delayMicroseconds(XSM_RST_SPEED); 
            Xlimit = digitalRead(XLimitPin);
        }
        XNsteps = 0;                //Location  X = 0 // MAX_HEIGHT*RATIO        
        digitalWrite(XdirPin,LOW); // Set Direction: Left

        for (int i=0 ; i<XMID_STEPS; i++) {
            digitalWrite(XstepPin,HIGH); 
            delayMicroseconds(XSM_RST_SPEED); 
            digitalWrite(XstepPin,LOW); 
            delayMicroseconds(XSM_RST_SPEED);  
            XNsteps+=1;
        }

        th1X = 90;                  // This is 90 degrees location from X-Axis
        digitalWrite(enable,HIGH); // disable motor
        Serial.println("Done X location setup");
    break;
    case SM_Y:
    Serial.println("Start Y location setup");
        digitalWrite(enable,LOW);
        digitalWrite(YdirPin,LOW); // Go Left
        while (Ylimit)
        {
            digitalWrite(YstepPin,HIGH); 
            delayMicroseconds(YSM_RST_SPEED); 
            digitalWrite(YstepPin,LOW); 
            delayMicroseconds(YSM_RST_SPEED); 
            Ylimit = digitalRead(YLimitPin);
        }
        YNsteps = 0;                //Location  X = 0 // MAX_HEIGHT*RATIO        
        digitalWrite(enable,LOW);   // Disable Step motors     
        digitalWrite(YdirPin,HIGH); // Set Direction: right

        for (int i=0 ; i<YMID_STEPS; i++) {
            digitalWrite(YstepPin,HIGH); 
            delayMicroseconds(YSM_RST_SPEED); 
            digitalWrite(YstepPin,LOW); 
            delayMicroseconds(YSM_RST_SPEED);  
            YNsteps+=1;
        }
        YNsteps_relative = YNsteps;
        digitalWrite(enable,HIGH);  // disable motor
        th2Y = 0;                   // This is 0 degrees location from L1 axis
        Serial.println("Done Y location setup");
    break;

    case SM_Z:
        Serial.println("Start Z location setup");
        digitalWrite(enable,LOW);
        digitalWrite(ZdirPin,LOW); // GO UP

        while (Zlimit)
        {
            digitalWrite(ZstepPin,HIGH); 
            delayMicroseconds(ZSM_RST_SPEED); 
            digitalWrite(ZstepPin,LOW); 
            delayMicroseconds(ZSM_RST_SPEED); 
            Zlimit = digitalRead(ZLimitPin);
        }
        height = MAX_HEIGHT;
        ZNsteps = 0;                //Location  Z = 0 // MAX_HEIGHT*RATIO        
        digitalWrite(enable,LOW);   // Disable Step motors     
        digitalWrite(ZdirPin,HIGH); // Set Direction: DOWN

        for (int i=0 ; i<MID_STEPS; i++) {
            digitalWrite(ZstepPin,HIGH); 
            delayMicroseconds(ZSM_RST_SPEED); 
            digitalWrite(ZstepPin,LOW); 
            delayMicroseconds(ZSM_RST_SPEED);  
            ZNsteps-=1;
        }
        zAxis  = ZNsteps*DIST_STEP_RATIO; 
        height = MAX_HEIGHT + zAxis;
        Serial.print("Mid location: ZNsteps:");    
        Serial.print(ZNsteps); 
        Serial.print(" zAxis[mm]:");
        Serial.print(zAxis); 
        Serial.print(" Heigt[mm]:");
        Serial.println(height); 
        digitalWrite(enable,HIGH); // disable motor

        Serial.println("Done Z location setup");

        break;

    default:
    Serial.println("Invalid resetStepMotor Command");
    break;
    }

}


void x_motor_step(){
    //move single step 
    digitalWrite(XstepPin,HIGH); 
    delayMicroseconds(XSMSpeed); 
    digitalWrite(XstepPin,LOW); 
    delayMicroseconds(XSMSpeed); 
}

void y_motor_step(){
    //move single step 
    digitalWrite(YstepPin,HIGH); 
    delayMicroseconds(YSMSpeed); 
    digitalWrite(YstepPin,LOW); 
    delayMicroseconds(YSMSpeed); 
}



void joy_xy_mode(bool save) {
// Joystic mode for XY dim
    bool cont = true;
    print_once = false;
    if (save) {pathIndex = 0;} // reset path mememory // todo change to link list
    Serial.println("");       
    while (cont) {
        vrx_data = analogRead(vrx);
        vry_data = analogRead(vry);
        Xlimit = digitalRead(XLimitPin);
        Ylimit = digitalRead(YLimitPin);

        // Stop state 
        if (((vrx_data > MID_TH-10)&&(vrx_data < MID_TH+10))&&
            ((vry_data > MID_TH-10)&&(vry_data < MID_TH+10)))
        {            
            
            if (print_once) {   
                digitalWrite(enable,HIGH); // STOP All motors
                if (save) { //save location
                        if (pathIndex < MAX_PATH_POINTS) {
                            path[pathIndex].theta1 = th1X;
                            path[pathIndex].theta2 = th2Y;
                            pathIndex++;
                        } else {
                            Serial.println("Warnning Path storage is full.");
                        }

                }
                Serial.print("Stop");
                Serial.print("| XSpeed:");
                Serial.print(XSMSpeed);
                Serial.print("| YSpeed:");
                Serial.print(YSMSpeed);
                Serial.print("| ZSpeed:");
                Serial.print(ZSMSpeed);            
                Serial.print(" ZNsteps:");
                Serial.print(ZNsteps); 
                Serial.print(" Heigt:");
                Serial.print(height); 
                Serial.print("z"); 
                Serial.print(" XNsteps:");
                Serial.print(XNsteps); 
                Serial.print(" Rel YNsteps:");
                Serial.print(YNsteps_relative); 
                Serial.print("|   Th1x: ");
                Serial.print(th1X); 
                Serial.print(" | Th2y: ");
                Serial.println(th2Y); 
                print_once = false; 
            }

        }

        //-----------------------------------------------------------------------------------------------------
        //                                                  X Axis Change
        //-----------------------------------------------------------------------------------------------------
        if ((vrx_data > UP_TH)&&(th1X < XMRANGE)&&(th2Y > -YMRANGE_XMOVE)) //X left if limit is not reached
        {
            digitalWrite(enable,LOW);
            digitalWrite(XdirPin,HIGH);        
            for (int i = 0 ; i< JRP; i++) {
                XNsteps-=1; 
                x_motor_step();
                Xlimit = digitalRead(XLimitPin);
                if (!Xlimit) continue;
            }
            YNsteps_relative = YNsteps_relative - JRP*XY_RATIO; //Reduce YNSTEP
            th1X -= JRP*XPROX_DEG_STEP;                         //TH1 Degree change
            th2Y -= JRP*YXDIST_DEG_STEP;                         //TH2 Relative Degree change

            print_once = true;
            xSpUpdate  = true;
        }
        if ((vrx_data < DOWN_TH)&&(th1X>-XMRANGE)&&(th2Y < YMRANGE_XMOVE)) //Right till allowed step delta
        {
            digitalWrite(enable,LOW);
            digitalWrite(XdirPin,LOW);        
            for (int i = 0 ; i< JRP; i++) {
                XNsteps+=1;
                x_motor_step();  
            }
            YNsteps_relative = YNsteps_relative + JRP*XY_RATIO; //Increase YNSTEP
            th1X += JRP*XPROX_DEG_STEP;
            th2Y += JRP*YXDIST_DEG_STEP;                         //TH2 Relative Degree change

            print_once = true;
            xSpUpdate  = true;
        }
        //-----------------------------------------------------------------------------------------------------
        //                                                  Y Axis Change
        //-----------------------------------------------------------------------------------------------------
        if ((vry_data > UP_TH)&&(th2Y > -YMRANGE)) //Up if limit is not reached
        {
            digitalWrite(enable,LOW);
            digitalWrite(YdirPin,LOW);
            for (int i = 0 ; i< JRP; i++) {
                YNsteps-=1; 
                y_motor_step();
                Ylimit = digitalRead(YLimitPin);
                if (!Ylimit) continue;
            }
            YNsteps_relative = YNsteps_relative - JRP;
            th2Y -= JRP*YDIST_DEG_STEP;
            print_once = true;
            ySpUpdate  = true;

        }
        if ((vry_data < DOWN_TH)&&(th2Y < YMRANGE)) // TODO add limit
        {
            digitalWrite(enable,LOW);
            digitalWrite(YdirPin,HIGH);
            for (int i = 0 ; i< JRP; i++) {
                YNsteps+=1;
               y_motor_step(); 
            }
            YNsteps_relative = YNsteps_relative + JRP; //Increase YNSTEP
            th2Y += JRP*YDIST_DEG_STEP;
            print_once = true;
            ySpUpdate  = true;
        }
        if ((analogRead(sw) == 0)&&((vry_data > MID_TH-10)&&(vry_data < MID_TH+10))) {
            if (isButtonPressed()) {
                cont = false; // exit mode when JS press
                Serial.println("Exit Joy XY mode ");
            }
        }        
    }
}



void joy_z_mode() {
// Joystic mode for Z dim
    bool cont = true;
    Serial.println(""); 
    while (cont) {
        ZSMSpeed = ZJSPEED; // move to joy speed
        vrz_data = analogRead(vry);
        Zlimit = digitalRead(ZLimitPin);

        // Stop state 
        if ((vrz_data > MID_TH-10)&&(vrz_data < MID_TH+10))
        {   
            
            if (print_once) {   
                digitalWrite(enable,HIGH); // STOP All motors
                Serial.println(""); 
                Serial.print("Stop");       
                Serial.print(" ZNsteps:");
                Serial.print(ZNsteps); 
                Serial.print(" Heigt:");
                Serial.println(height); 
                print_once = false; 
            }
        }
        //-----------------------------------------------------------------------------------------------------
        //                                                Z Axis Change
        //-----------------------------------------------------------------------------------------------------
        if (Zlimit &&(vrz_data > UP_TH)) //Up if limit is not reached
        {
            digitalWrite(enable,LOW);
            digitalWrite(ZdirPin,LOW);
            ZNsteps+=1; 
            zAxis  = ZNsteps*DIST_STEP_RATIO; 
            height = MAX_HEIGHT + zAxis;
            //Serial.print("Up:");
            //Serial.println(vrx_data); 
            digitalWrite(ZstepPin,HIGH); 
            delayMicroseconds(ZSMSpeed); 
            digitalWrite(ZstepPin,LOW); 
            delayMicroseconds(ZSMSpeed); 
            print_once = true;
            zSpUpdate  = true;    
        }
        if ( (vrz_data < DOWN_TH)&&(height>MIN_HEIGHT)) //down till min heigt reached
        {
            //Serial.print("Down:");
            //Serial.println(vrx_data);
            digitalWrite(enable,LOW);
            digitalWrite(ZdirPin,HIGH);
            ZNsteps-=1;
            zAxis  = ZNsteps*DIST_STEP_RATIO; 
            height = MAX_HEIGHT + zAxis;
            digitalWrite(ZstepPin,HIGH); 
            delayMicroseconds(ZSMSpeed); 
            digitalWrite(ZstepPin,LOW); 
            delayMicroseconds(ZSMSpeed);  
            print_once = true;
            zSpUpdate  = true;
        }
        if ((analogRead(sw) == 0)&&((vrz_data > MID_TH-10)&&(vrz_data < MID_TH+10))) {
            if (isButtonPressed()) {
                cont = false; // exit mode when JS press       
                Serial.println("Exit Joy z mode ");
            }
        }

    }
}


void calculateInverseKinematics(float x, float y, float *theta1, float *theta2) {
  Serial.println("Calculate inverse kinematic"); 
  bool isXneg = (x<0.0);  
  x = fabs(x);
  float cosalpha = (L1*L1 + L2*L2 - x*x - y*y)/(2*L1*L2);
  *theta2 = (PI - acos(cosalpha));
  float cosTheta2 = cos(*theta2);
  float sinTheta2 = sin(*theta2);
  *theta1 = atan(y/x) - atan(L2 * sinTheta2 / (L1 + L2 * cosTheta2));

  *theta2 = *theta2 * 180.0 / PI; // Convert to degrees
  *theta1 = *theta1 * 180.0 / PI; // Convert to degrees

  if (isXneg) {
    *theta1 = 180 - *theta1;
    *theta2 = -*theta2;
  }  

  Serial.println("=====================================================");
  Serial.print("theta1:");
  Serial.print(*theta1);
  Serial.print(" theta2:");
  Serial.println(*theta2);
  Serial.print(" sinTheta2:");
  Serial.print(sinTheta2);
  Serial.print(" CosTheta2:");
  Serial.println(cosTheta2);
  Serial.println("=====================================================");
  // Apply the angle ratio to theta2
  //*theta2 = *theta2 * angleRatio;
}

void goto_xy(long x, long y) {
        int dir;
        Serial.println("Goto x y"); 
        float xf = (float)x;
        float yf = (float)y;
        float theta1, theta2;

     Serial.print("X=");
     Serial.println(xf);
     if (isReachable(xf, yf, L1, L2)) {
        calculateInverseKinematics(xf, yf, &theta1, &theta2);        
        Serial.print("theta1:");
        Serial.print(theta1);
        Serial.print(" theta2:");
        Serial.println(theta2);
        Serial.print("th1X:");
        Serial.print(th1X);
        Serial.print(" th2Y:");
        Serial.println(th2Y);

        //Adjust arm proximal location
        digitalWrite(enable,LOW);
        if ((th1X - theta1) > 0 ) {
            digitalWrite(XdirPin,HIGH);
            dir = 1;
        } else {
            digitalWrite(XdirPin,LOW);  
            dir = 0;
        }
        while  ((fabs(th1X - theta1)>THETA_TH)&&(fabs(th1X)<XMRANGE)) {
            for (int i=0;i<DX;i++) {
                x_motor_step();
            }
            if (dir==1) {
                th1X -= XPROX_DEG_STEP*DX;
                th2Y += YXDIST_DEG_STEP*DX;
            }else {
                th1X += XPROX_DEG_STEP*DX;         //TH1 Degree change
                th2Y -= YXDIST_DEG_STEP*DX;
            }
            //Serial.print("th1X:");
            //Serial.println(th1X);
        }

        digitalWrite(enable,LOW);
        if ((th2Y - theta2) > 0 ) {
            digitalWrite(YdirPin,HIGH);
            dir = 0;
        } else {
            digitalWrite(YdirPin,LOW);  
            dir = 1;
        }
        while  ((fabs(th2Y - theta2)>THETA_TH)) {//&& (fabs(th2Y)<=YMRANGE)*!dir) {
            for (int i=0;i<DX;i++) {
                y_motor_step();
            }
            if (dir==0) th2Y -= YDIST_DEG_STEP*DX;else th2Y += YDIST_DEG_STEP*DX;         //TH1 Degree change
            //Serial.print("th2Y:");
            //Serial.println(th2Y);
        }
        Serial.print("th1X:");
        Serial.print(th1X);
        Serial.print(" th2Y:");
        Serial.println(th2Y);        
     } else {
        Serial.print("This location is not reachable due arm physical constrains!");

     }
        digitalWrite(enable,HIGH); // disable all motors

}

bool isReachable(float x, float y, float L1, float L2) {
  if (y < 0) return false;
  float distance = sqrt(x * x + y * y);
  return (distance <= (L1 + L2) && distance >= fabs(L1 - L2));
}

void load_path_xy(int idx) {
    //replay recorded path idx times
    Serial.print("Replay recorded path N:");
    Serial.print(idx);
    Serial.println(" times");
    Serial.print("Path length:");
    Serial.println(pathIndex);

    for (int i = 0;i<idx ; i++){

        // Replay backward
        Serial.print("idx:");
        Serial.println(i);

        for (int j = pathIndex-1 ;j >= 0; j--) {
            Serial.print("Point path:");
            Serial.println(j);
            moveArmTo(path[j].theta1, path[j].theta2);
            delay(1000); // Adjust delay as needed
        }
        // Play forward
        for (int j = 1;j < pathIndex;j++) {
            Serial.print("Point path:");
            Serial.println(j);
            moveArmTo(path[j].theta1, path[j].theta2);
            delay(1000); // Adjust delay as needed
        }

    }

}

void moveArmTo(float theta1, float theta2) {
  // move arms to requested th1 and th2
    Serial.print("Current Th1X:");
    Serial.print(th1X);
    Serial.print(" Moving to Theta 1: ");
    Serial.print(theta1);
    Serial.print(" | Theta 2: ");
    Serial.println(theta2);
    //Adjust arm proximal location - Step motor X
    moveArmToTh1(theta1);
    //Adjust arm Distal   location - Step motor Y
    moveArmToTh2(theta2);
   
    Serial.print("Final th1X:");
    Serial.print(th1X);
    Serial.print(" th2Y:");
    Serial.println(th2Y);     
}

void moveArmToTh1(float theta1) {
//Adjust arm proximal location
    int dir = 0;
    digitalWrite(enable,LOW);
    if ((th1X - theta1) > 0 ) {
        digitalWrite(XdirPin,HIGH);
        dir = 1;
    } else {
        digitalWrite(XdirPin,LOW);  
        dir = 0;
    }
    while  ((fabs(th1X - theta1)>THETA_TH)&&(fabs(th1X)<XMRANGE)) {
            for (int i=0;i<DX;i++) {
                x_motor_step();
            }
            if (dir==1) {
                th1X -= XPROX_DEG_STEP*DX;
                th2Y -= YXDIST_DEG_STEP*DX;
            }else {
                th1X += XPROX_DEG_STEP*DX;         //TH1 Degree change
                th2Y += YXDIST_DEG_STEP*DX;
            }
    }

}

void moveArmToTh2(float theta2) {
    int dir = 0;
    digitalWrite(enable,LOW);
    if ((th2Y >0)&&(th2Y - theta2) > 0 ) {
        digitalWrite(YdirPin,LOW);
        dir = 0;
    } else if ((th2Y>0)&&((th2Y - theta2) < 0 )) {
        digitalWrite(YdirPin,HIGH);  
        dir = 1;      
    } else if ((th2Y<0)&&((th2Y > theta2))){
        digitalWrite(YdirPin,LOW);  
        dir = 0;
    } else if ((th2Y<0)&&((th2Y < theta2))){
        digitalWrite(YdirPin,HIGH);  
        dir = 1;
    }

    while  ((fabs(th2Y - theta2)>THETA_TH)) {//&& (fabs(th2Y)<=YMRANGE)*!dir) {
        for (int i=0;i<DX;i++) {
            y_motor_step();
        }
        if (dir==0) th2Y -= YDIST_DEG_STEP*DX;else th2Y += YDIST_DEG_STEP*DX;         //TH1 Degree change
    }

}

void  arm_location() {
// Return Arm location
    Serial.println("-------------------------------");
    Serial.print("th1X:");
    Serial.print(th1X);
    Serial.print(" th2Y:");
    Serial.println(th2Y);
    Serial.println("-------------------------------");

}
