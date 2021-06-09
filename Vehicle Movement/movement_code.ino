//Left Side Motor
const int EnableL=5;
const int HighL=6;
const int LowL=7;

//Right Side Motor
const int EnableR=10;
const int HighR=8;
const int LowR=9;

const int D0=0;
const int D1=1;
const int D2=2;
const int D3=3;

int a,b,c,d,data;
void Data()
{
  a= digitalRead(D0);
  b= digitalRead(D1);
  c= digitalRead(D2);
  d= digitalRead(D3);

  data=8*d+c*4+b*2+a*1;
  
}

void setup() {
 

  pinMode(EnableL,OUTPUT);
  pinMode(HighL,OUTPUT);
  pinMode(EnableL,OUTPUT);

  pinMode(EnableR,OUTPUT);
  pinMode(HighR,OUTPUT);
  pinMode(EnableR,OUTPUT);

  pinMode(D0,INPUT_PULLUP); //INPUT_PULLUP
  pinMode(D1,INPUT_PULLUP);
  pinMode(D2,INPUT_PULLUP);
  pinMode(D3,INPUT_PULLUP);



}
void Forward()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,255);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,255);
}

void Stop()
{ 

  
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);


}

void Left1()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,225);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,255);
}

void Left2()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,200);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,255);
}

void Left3()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,175);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,255);
}

void Right1()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,255);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW );
  analogWrite(EnableR,225);
}

void Right2()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,255);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,200);
}

void Right3()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,255);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,175);
}




void Stopd()
{
    
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  delay(1000);
  }  

void loop() {
  Data();
 
  
    if(data==0)
    {
      Forward();
      }
      
    else if  (data==7 )
    {
      Stop();
      }
    else if  (data==8 )
    {
      Stopd();
      }
    else if(data==1)
    { Left1();
    }
    else if(data==2)
    { Left2();
    }
    else if(data==3)
    { Left3();
    }
    else if(data==4)
   { Right1();
   }
   else if(data==5)
    { Right2();
   }
   else if(data==6)
    { Right3();
   }
   
}
