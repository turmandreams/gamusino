

int valorcon[4];
int valorsin[4];


void setup(){

    Serial.begin(115200);

    pinMode(36,INPUT);
    pinMode(39,INPUT);
    pinMode(34,INPUT);
    pinMode(35,INPUT);
    pinMode(32,OUTPUT);
    pinMode(33,OUTPUT);

    
}


void loop(){

    digitalWrite(32,LOW);
    digitalWrite(33,LOW);

    delay(1);

    
    valorsin[0]=analogRead(36);
    valorsin[1]=analogRead(39);
    valorsin[2]=analogRead(34);
    valorsin[3]=analogRead(35);

    
    digitalWrite(32,HIGH);
    digitalWrite(33,HIGH);

    delay(1);

    valorcon[0]=analogRead(36);
    valorcon[1]=analogRead(39);
    valorcon[2]=analogRead(34);
    valorcon[3]=analogRead(35);


    Serial.print((valorcon[0]-valorsin[0]));
    Serial.print(",");
    Serial.print((valorcon[1]-valorsin[1]));
    Serial.print(",");
    Serial.print((valorcon[2]-valorsin[2]));
    Serial.print(",");
    Serial.println((valorcon[3]-valorsin[3]));

    delay(50);
}
