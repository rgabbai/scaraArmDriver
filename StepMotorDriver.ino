


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



}


void resetStepMotor(char smNum) {
// Reset motor location using limit micro switch

    switch(smNum) {
    case SM_Z:
        Serial.println("Start Z location setup");
        digitalWrite(enable,LOW);
        digitalWrite(ZdirPin,LOW); // GO UP

        while (Zlimit)
        {
            digitalWrite(ZstepPin,HIGH); 
            delayMicroseconds(ZSM_SPEED); 
            digitalWrite(ZstepPin,LOW); 
            delayMicroseconds(ZSM_SPEED); 
            Zlimit = digitalRead(ZLimitPin);
        }
        height = MAX_HEIGHT;
        ZNsteps = 0;                //Location  Z = 0 // MAX_HEIGHT*RATIO        
        digitalWrite(enable,LOW);   // Disable Step motors     
        digitalWrite(ZdirPin,HIGH); // Set Direction: DOWN

        for (int i=0 ; i<MID_STEPS; i++) {
            digitalWrite(ZstepPin,HIGH); 
            delayMicroseconds(ZSM_SPEED); 
            digitalWrite(ZstepPin,LOW); 
            delayMicroseconds(ZSM_SPEED);  
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

void joy_xy_mode() {
// Joystic mode for XY dim
    bool cont = true;

    while (cont) {
        vrx_data = analogRead(vrx);
        vry_data = analogRead(vry);



        // Stop state 
        if (((vrx_data > MID_TH-10)&&(vrx_data < MID_TH+10))&&
            ((vry_data > MID_TH-10)&&(vry_data < MID_TH+10)))
        {            
            
            if (print_once) {   
                digitalWrite(enable,HIGH); // STOP All motors

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
                Serial.println(height); 
                Serial.println("z"); 
                print_once = false; 
            }

        }

        //-----------------------------------------------------------------------------------------------------
        //                                                  X Axis Change
        //-----------------------------------------------------------------------------------------------------
        if (Xlimit &&(vrx_data > UP_TH)) //X left if limit is not reached
        {
            digitalWrite(enable,LOW);
            digitalWrite(XdirPin,LOW);        
            for (int i = 0 ; i< RP; i++) {
                XNsteps+=1; 
                //xAxis  = TBD 
                digitalWrite(XstepPin,HIGH); 
                delayMicroseconds(XSMSpeed); 
                digitalWrite(XstepPin,LOW); 
                delayMicroseconds(XSMSpeed); 
            }
            print_once = true;
            xSpUpdate  = true;
        }
        if ( (vrx_data < DOWN_TH))//&&(XNsteps>MINXNsteps)) //Right till allowed step delta
        {
            digitalWrite(enable,LOW);
            digitalWrite(XdirPin,HIGH);        
            for (int i = 0 ; i< RP; i++) {

                XNsteps-=1;
                //xAxis  = TBD
                digitalWrite(XstepPin,HIGH); 
                delayMicroseconds(XSMSpeed); 
                digitalWrite(XstepPin,LOW); 
                delayMicroseconds(XSMSpeed);  
            }
            print_once = true;
            xSpUpdate  = true;
        }
        //-----------------------------------------------------------------------------------------------------
        //                                                  Y Axis Change
        //-----------------------------------------------------------------------------------------------------
        if (Ylimit &&(vry_data > UP_TH)) //Up if limit is not reached
        {
            digitalWrite(enable,LOW);
            digitalWrite(YdirPin,LOW);
            for (int i = 0 ; i< RP; i++) {
                YNsteps+=1; 
                //yAxis  = TBD
                digitalWrite(YstepPin,HIGH); 
                delayMicroseconds(YSMSpeed); 
                digitalWrite(YstepPin,LOW); 
                delayMicroseconds(YSMSpeed); 
            }
            print_once = true;
            ySpUpdate  = true;

        }
        if ((vry_data < DOWN_TH)) // TODO add limit
        {
            digitalWrite(enable,LOW);
            digitalWrite(YdirPin,HIGH);
            for (int i = 0 ; i< RP; i++) {
                YNsteps-=1;
                //yAxis  = TBD
                digitalWrite(YstepPin,HIGH); 
                delayMicroseconds(YSMSpeed); 
                digitalWrite(YstepPin,LOW); 
                delayMicroseconds(YSMSpeed);  
            }
            print_once = true;
            ySpUpdate  = true;
        }
        if ((analogRead(sw) == 0)&&((vrz_data > MID_TH-10)&&(vrz_data < MID_TH+10))) {
            cont = false; // exit mode when JS press
            Serial.println("Exit Joy xy mode ");
        }        
    }
}



void joy_z_mode() {
// Joystic mode for Z dim
    bool cont = true;

    while (cont) {
        ZSMSpeed = ZJSPEED; // move to joy speed
        vrz_data = analogRead(vry);
        Zlimit = digitalRead(ZLimitPin);

        //Serial.println("");       
        //Serial.print("vrz_data:");       
        //Serial.println(vrz_data);       


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
            cont = false; // exit mode when JS press
            Serial.println("Exit Joy z mode ");
        }

    }
}