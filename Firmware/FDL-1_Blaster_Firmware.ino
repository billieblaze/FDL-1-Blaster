SYSTEM_MODE(SEMI_AUTOMATIC);

// Define the pins we're going to call pinMode on
int advSenseIn = D0;
int plungerSenseIn = D1;
int dartInChamberIn = D2;
int irPulseOut = D3;

int stepperAdvanceStep = D5;
int stepperFireStep = D4;
int stepperEnable = D6;
int blasterStepperDir = D7;

int speedSenseIn = A2;
int triggerSenseIn = A3;//TX;
int advanceButton = A4;

int connSenseIn = RX;
int escPin = TX;

int dartInChamberSwitch = DAC;
int chamberSwitchPower = WKP;

Servo flywheelESC;  // create servo object to control a ESC

double speedValue = 0.0;
bool firstRun = true;

unsigned long disableMillis = millis();

int stepperWarmup = 80;
int advanceSpeed = 380; 
unsigned long flywheelSpinup = 400;


// This routine runs only once upon reset
void setup() {
    
  flywheelESC.attach(escPin);  // attaches pin to servo object
    
  // Initialize pins 
  // It's important you do this here, inside the setup() function rather than outside it or in the loop function.
  pinMode(stepperAdvanceStep, OUTPUT);
  pinMode(stepperFireStep, OUTPUT);
  pinMode(irPulseOut, OUTPUT);
  pinMode(stepperEnable, OUTPUT);
  
  pinMode(advSenseIn, INPUT_PULLDOWN);
  pinMode(plungerSenseIn, INPUT_PULLDOWN);
  pinMode(triggerSenseIn, INPUT_PULLDOWN);
  pinMode(connSenseIn, INPUT_PULLDOWN);
  pinMode(dartInChamberSwitch, INPUT_PULLDOWN);
  pinMode(advanceButton, INPUT_PULLDOWN);
  pinMode(dartInChamberIn, INPUT_PULLDOWN);
  
  pinMode(speedSenseIn, INPUT);
  
  pinMode(chamberSwitchPower, OUTPUT);
  digitalWrite(chamberSwitchPower, HIGH);
  
  pinMode(blasterStepperDir, OUTPUT);
  digitalWrite(blasterStepperDir, LOW);
  
  digitalWrite(stepperEnable, HIGH); // Turn off steppers (HIGH)
}

// This routine gets called repeatedly, like once every 5-15 milliseconds.
// Spark firmware interleaves background CPU activity associated with WiFi + Cloud activity with your code. 
// Make sure none of your code delays or blocks for too long (like more than 5 seconds), or weird things can happen.
void loop() {
  
    if(digitalRead(triggerSenseIn) == HIGH){
        
        if(firstRun){
            while(digitalRead(triggerSenseIn) == HIGH){
                flywheelESC.write(180); 
            }
        
            flywheelESC.write(0); 
            delay(1500);  
        }
        else{
            int val = analogRead(speedSenseIn);
            val = map(val, 0, 4094, 60, 130);
            fireBrushlessLoop(val);
        }
    }
    else{
        flywheelESC.write(0); 
    }
    
    if(digitalRead(connSenseIn) == HIGH){
      Particle.connect();
    }
    
    firstRun = false;
    
    delay(10);
    
    if(millis() > disableMillis){
        digitalWrite(stepperEnable, HIGH);
    }
    
    if(digitalRead(advanceButton) == HIGH){
      advanceWithEnableDisable();
    }
    
    speedValue = analogRead(speedSenseIn);
}

void advanceWithEnableDisable(){
    
    digitalWrite(stepperEnable, LOW);
    delay(stepperWarmup);
           
    advance();
    
    delay(40);
    //sets disable time
    disableMillis = millis() + 300;
}

boolean advance(){
    
    bool useDartInChamber = dartInChamberEnabled();
    
    return advanceWithFlag(useDartInChamber);
}

boolean advanceWithFlag(bool useDartInChamber){
    
    if(!useDartInChamber){
        return advanceSub();
    }
    else{
        int tryCount = 1;
        
        while(tryCount < 12){
            
            if(digitalRead(advSenseIn) == HIGH && isDartInChamber() == true){
                return true;
            }
            
            if(advanceSub()){
                //delay(50);
                
                int val = analogRead(dartInChamberIn);
                if(isDartInChamber() == true){
                    return true;
                }
            }
            else{
                return false;
            }
            tryCount++;
        }
    }
    
    return false;
}

boolean advanceSub(){
    
    //133.33 steps per chamber
    
    int senseTest = digitalRead(advSenseIn); 
    
    int successCheck = 0;
    
    int advanceSpeedLocal = advanceSpeed;
    
    
    if(senseTest == HIGH){
        StepRange(stepperAdvanceStep, 500, advanceSpeedLocal, 60);
        
        while(senseTest == HIGH){
            if(successCheck > 100){
                return false;
            }
            
            StepDelay(stepperAdvanceStep, advanceSpeedLocal, 1);
            senseTest = digitalRead(advSenseIn);
            
            successCheck++;
        } 
    }
    else{
        StepRange(stepperAdvanceStep, 500, advanceSpeedLocal, 20);
    }
    
    successCheck = 0;
    
    while(senseTest == LOW){
        if(successCheck > 200){
            return false;
        }
        
        StepDelay(stepperAdvanceStep, advanceSpeedLocal, 1);
        senseTest = digitalRead(advSenseIn);
        
        successCheck++;
        advanceSpeedLocal += 5;
        advanceSpeedLocal = min(800, advanceSpeedLocal);
    }
    
    return true;
}



boolean isDartInChamber(){
    
    for(int index = 0; index < 15; index++){
        digitalWrite(irPulseOut, HIGH);
        delayMicroseconds(13);
        digitalWrite(irPulseOut, LOW);
        delayMicroseconds(13);
    }
    
    if(digitalRead(dartInChamberIn) == HIGH){//ir blocked
        return true;
    }
    else{
        return false; 
    }
    
}

boolean dartInChamberEnabled(){
    return digitalRead(dartInChamberSwitch) == HIGH;
}







void fireBrushlessLoop(double powerVal){
    
    int flywheelSpinupExtraPower = 40;

    double spinupPower = powerVal;
    
    unsigned long spinupEnd = millis() + flywheelSpinup;
    
    //kick on flywheels
    flywheelESC.write(spinupPower); 
    
    //kick on steppers, wait for warmup
    digitalWrite(stepperEnable, LOW);
    delay(stepperWarmup);
    
    if(digitalRead(advSenseIn) == LOW || dartInChamberEnabled()){ 
        if(!advance()){ 
            brushlessPowerDown(1000);
            return;
        }
    }
    
    //still holding the trigger?
    if(digitalRead(triggerSenseIn) == HIGH){
        
        //wait for spinup
        while(millis() < spinupEnd ){ delay(1); }
        
        while(digitalRead(triggerSenseIn) == HIGH){
            
            spinPlungerToSwitch();
            
            if(digitalRead(triggerSenseIn) == HIGH){
                if(!advance()){ 
                    brushlessPowerDown(1000);
                    return;
                }
            }
            else{
                flywheelESC.write(0); 
                advance();
                brushlessPowerDown(1000);
                return;
            }
        }
    }
    else{ 
        brushlessPowerDown(1000);
        return;
    }
    
}

void brushlessPowerDown(double millisToDisable){
    
    flywheelESC.write(0); 
    
    //sets disable time for 1 sec
    disableMillis = millis() + millisToDisable;
}

bool spinPlungerToSwitch(){
    
    //if(digitalRead(advSenseIn) == LOW){ return false; }
    
    //1600 full spin
    //spin enough to let go of the switch (1/2 wayish)
    //StepRange(stepperFireStep, 180, 140, 100);
    StepRange(stepperFireStep, 400, 180, 100);
    StepRange(stepperFireStep, 140, 120, 800);

    for(int stepIndex = 0; stepIndex < 1600; stepIndex++){//500
       StepRange(stepperFireStep, 120, 120, 1); 
       
       if(digitalRead(plungerSenseIn) == HIGH){
           return true;
       }
    }
    
    return false;
}



void StepDelay(int stepperPin, double delayMicros, int steps){
     for (int index = 0 ; index < steps ; index ++) {
        digitalWrite(stepperPin, HIGH); 
        delayMicroseconds(delayMicros / 2); 
        digitalWrite(stepperPin, LOW); 
        delayMicroseconds(delayMicros / 2); 
    }
}

void StepRange(int stepperPin, double startDelay, double endDelay, double steps){
    
    double delayChangePerStep = (endDelay - startDelay) / steps;
    
    double loopDelay = startDelay;
    
    for (int index = 0 ; index < steps ; index += 1) {
        digitalWrite(stepperPin, HIGH); 
        delayMicroseconds(loopDelay / 2); 
        digitalWrite(stepperPin, LOW); 
        delayMicroseconds(loopDelay / 2); 
        
        loopDelay += delayChangePerStep;
    }
}

