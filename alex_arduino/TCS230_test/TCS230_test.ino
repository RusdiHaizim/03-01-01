#define TCS230_s0 7 //TCS 230 led
#define TCS230_s1 8 //TCS230 led
#define TCS230_s2 12 // TCS230 led
#define TCS230_s3 13 // TCS230 led 
#define TCS230_out 4 //input from TCS230
volatile int TCS230_frequency = 0; //TCS230 frequency
volatile int TCS230_red = 0, TCS230_green = 0, TCS230_blue = 0;

void setup_TCS230(){
  pinMode(TCS230_s0, OUTPUT);
  pinMode(TCS230_s1, OUTPUT);
  pinMode(TCS230_s2, OUTPUT);
  pinMode(TCS230_s3, OUTPUT);
  pinMode(TCS230_out,INPUT);
  //setting frequency scaling to 20%
  digitalWrite(TCS230_s0,HIGH);
  digitalWrite(TCS230_s1,LOW);  
}

void setup() {
  // put your setup code here, to run once:
  setup_TCS230();
  Serial.begin(9600);
}

void TCS230_run(){
  //set red filtered photodiodes to be read
  digitalWrite(TCS230_s2,LOW);
  digitalWrite(TCS230_s3,LOW);
  //read output frequency
  TCS230_red = pulseIn(TCS230_out,LOW);
  //map red 
//  TCS230_red = map(TCS230/_red, 100, 500, 255, 0);
  delay(100);  //check if time delay between each colour check is necessary
  
  //set green photodiodes to be read
  digitalWrite(TCS230_s2,HIGH);
  digitalWrite(TCS230_s3,HIGH);
  //read output frequency
  TCS230_green = pulseIn(TCS230_out,LOW);  
  //map green
//  TCS230_green = map(TCS230_green/,60,400,255,0);
  delay(100);
  
  //set blue filtered photodiodes to be read
  digitalWrite(TCS230_s2,LOW);
  digitalWrite(TCS230_s3,HIGH);
  //read output frequency
  TCS230_blue = pulseIn(TCS230_out,LOW);
  //map blue
//  TCS230_blue = map(TCS230_bl/ue, 85, 490, 255, 0);
  //print frequency values
  Serial.print("red ");
  Serial.println(TCS230_red);
  Serial.print("green ");
  Serial.println(TCS230_green);
  Serial.print("blue ");
  Serial.println(TCS230_blue);
  //if all values are close to each other, colour is green
  if (abs(TCS230_red - 255) < 30 && abs(TCS230_green - 255) < 30 && abs(TCS230_blue - 255) < 30){
    Serial.println("green"); 
  } else if (abs(TCS230_red - 255) < 30 && abs(TCS230_green - 255) > 70 && abs(TCS230_blue - 255) > 70){
    Serial.println("red"); 
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  TCS230_run();
}


