


#define CH_A_PIN 2
#define CH_B_PIN 3
volatile long count = 0;
//volatile int new_state = 0;
//volatile int old_state = 0;

void encoder_ISR(){

  int new_CHA = digitalRead(CH_A_PIN);
  int new_CHB = digitalRead(CH_B_PIN);

  int new_state = 0;
  new_state|= new_CHB << 1 ;
  new_state |= new_CHA;

  static int old_state = new_state;
  long dir = 1;
  if(old_state==0 && new_state == 1){ //forward sequence 0->1->3->2->0
    count+=dir;    
  }else if(old_state==1 && new_state == 3){
    count+=dir; 
  }else if(old_state==3 && new_state == 2){
    count+=dir; 
  }else if(old_state==2 && new_state == 0){
    count+=dir;    
  } else if(old_state==1 && new_state == 0){ ///reverse sequence 1->0->2->3->1
    count-=dir;    
  }else if(old_state==0 && new_state == 2){
    count-=dir; 
  }else if(old_state== 2 && new_state == 3){
    count-=dir; 
  }else if(old_state==3 && new_state == 1){
    count-=dir;    
  }
  old_state = new_state;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(CH_A_PIN, INPUT);
  pinMode(CH_B_PIN, INPUT);
  
  encoder_ISR();
  attachInterrupt(digitalPinToInterrupt(CH_A_PIN), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_B_PIN), encoder_ISR, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int CHA = digitalRead(CH_A_PIN);
  int CHB = digitalRead(CH_B_PIN);
  /*
  Serial.print(""); Serial.print(CHA);
  Serial.print(","); Serial.print(CHB);
  Serial.print(","); Serial.print(count);
  Serial.print(","); Serial.print(new_state);
  Serial.print(","); Serial.println(old_state);*/
  //Serial.print(","); Serial.println(count);

  delay(2);
}
