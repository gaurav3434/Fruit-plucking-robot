///////////////////////////////////// This code is for grabbing and placing the oranges sucessfully 
///////////////////////////////////// Also moves robot towards orange if its out of workspace.
#include <ESP32Servo.h> 
Servo myservo_base;  
Servo myservo_mid;  
Servo myservo_up;  
Servo myservo_turn;  
Servo myservo_grab;
Servo myservo_cut;
Servo myservo_1;
Servo myservo_2;
Servo myservo_3;
Servo myservo_4;

int orientation_angle=170;   // 170 for pick and place from table
int Serial_input;
int error=0;  // diff btn object positon and camera center 
int Move=0;   // decides direction of movement for camera's center // 0 means go home position 
int Width=1400;    // object width 
float x=0.0;    // for speed_map function
float y=0.0;    // for speed_map function
float margin_with_center=30.0;  // for speed_map function
float margin_proximity=30.0;    // for speed_map function
float Speed_forward_reverse=0; // dependent on width 
float Speed_V=0.3;  // 10.3 max // changes EndX,EndY values 
float Speed_H=0.2; // 7.2 max // changes direct motor output value 
float L1=120;   // link 1st length
float L2=170;   // link 2nd length
float Theta_1;  // base motor
float Theta_2;  // mid motor
float Theta_3;  // up motor
float Theta_turn=90;  // turn motor
float EndX=1;      // input -200 to +230 for smooth linear motion @ Y=+170
float EndY=130;    // input -170 to +260 for smooth linear motion @ X=+120
float Previous_EndX= EndX; // for checking if new values of EndX & EndY are in workspace 
float Previous_EndY= EndY; // 
float phi_1,phi_2;
long wait_time=0;
bool Home_var= true;
int wait=1;  // 1: out of grabbing range, 2: in grabbing range, 3: entered in counting.
int Step=1; //1=serial_data()&&motion(), 2 = pick_and_place() 
int look_out_var=0; 
float i=0.0;
void setup()
{
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo_base.setPeriodHertz(50);// Standard 50hz servo
  myservo_mid.setPeriodHertz(50);
  myservo_up.setPeriodHertz(50);
  myservo_turn.setPeriodHertz(50);
  myservo_grab.setPeriodHertz(50);
  myservo_cut.setPeriodHertz(50);
  myservo_1.setPeriodHertz(50);
  myservo_2.setPeriodHertz(50);
  myservo_3.setPeriodHertz(50);
  myservo_4.setPeriodHertz(50);
  
  myservo_base.attach(13, 550, 2450); // base/ 550/ 2450   
  myservo_mid.attach(12, 550, 2450);  // mid/ 550 / 2450
  myservo_up.attach(15, 500, 2500);   // up/ 500 / 2500
  myservo_turn.attach(14, 500, 2170); // turn/ 500 / 2170  // between 50, 130  angle 
  myservo_grab.attach(2, 500, 2500); // 110: open, 50: tight grab
  myservo_cut.attach(4,500,2500);   
  myservo_1.attach(5,500,2500); 
  myservo_2.attach(18,500,2500);     
  myservo_3.attach(19,500,2500);     
  myservo_4.attach(23,500,2500);     
  Serial.begin(115200);
  myservo_grab.write(120); // grab=90, open=120
}
void loop()
{if (Step==3) 
    {look_out();
     Move_robot(0); 
    } 
 if (Step==1)        // Home / serial_data()/ motion/
    { serial_data();
      motion(); }
 if (Step==2)         //grab/pick and place
    { pick_and_place(); 
      Move_robot(0); }     
}     
                     /////////////////////////////////////////////////        *******         //////////////////////////////////////

void serial_data()
{ 
  if (Serial.available()>0)
     { String str= Serial.readStringUntil('|');
       Serial_input= (str.toInt()); 
       if (Serial_input>1400) // its width then 
          { Width=round((Width*5/10)+(Serial_input*5/10)); 
            float error_1=abs(1800-Width);
                if (Width<1800)
                   { Move=5;} // move_forward 
                if (Width<1700)
                   { wait=1;}    
                if (wait==1)
                   {wait_time=millis();}   // out of grabbing range
                if (Width>1800 && Speed_V<=1.8)  // means object is in right spot to grab
                   { Move=6;  // move back
                     wait=2;  // in grabbing range 
                     Move_robot(0);}
                if ((EndX>120 && (sqrt(sq(EndX)+sq(EndY)))<284) || (EndX<120 && Width<=1600)) 
                    { Move_robot(0); }     
                if (Width>1650 && Speed_V>3.0 && EndX<120) // means object is too near to robot in order to reach it
                   { //Move=6;  // move back
                     Move_robot(2); }// Move wheels back
                Speed_forward_reverse= speed_map(error_1,400,0,6.3,0.0);
          }
      if (Width<1700)
         { wait=1;}      
      if (wait==2 && (millis()-wait_time)>1000)
         { Step=2;} 
      if (Serial_input>800 && Serial_input<1400)
         { error=abs(1100-Serial_input); 
           if (Serial_input>1100) 
          
              { Move=4;} // 4 means down
           if (Serial_input<1100)
              { Move=2;} // 2 means up
           Speed_V= speed_map(error,300,margin_with_center,10.3,0.0); 
         }    
//                                  ///////////////////   move up down ////////////////
      if (Serial_input<800 && Serial_input>1)
         { error=abs(480-Serial_input); 
           if (Serial_input>480)  
          
              { Move=1;} // 4 means left
           if (Serial_input<480)
              { Move=3;} // 2 means right
           Speed_H= speed_map(error,400,margin_with_center,6.5,0.0); 
         }    
      if (Serial_input==0) // Nothing detected
         { Move=0; }    
  ///////////////////////////////////////////////    move _ Left/ Right/ up/ down/ forward/ reverse/ home  //////////
  float Previous_EndX= EndX;
  float Previous_EndY= EndY;
      if (Move==4) // down
         {EndY=EndY-Speed_V;} 
      if (Move==2) // up
         {EndY=EndY+Speed_V;}
      if (Move==1) // left 
         {Theta_turn=Theta_turn-Speed_H;} 
      if (Move==1 && Theta_turn<49) // left 
         { Move_robot(3);}  
      if (Move==3) // right 
         {Theta_turn=Theta_turn+Speed_H;}
      if (Move==3 && Theta_turn>129) // left 
         { Move_robot(4);}   
      Home_var=false;   
      if (Move==0) // Nothing detected 
         { Step=3;
           look_out_var=0;
           Home_var=true;  // true to go home position 
           Home();
           Width=1400;
           wait_time=millis();
           Open();
           Move_robot(0);
         }     
      if (Move==5) // move front 
         {EndX= EndX+Speed_forward_reverse;}   
      if (Move==6) // move back 
         {EndX= EndX-Speed_forward_reverse;} 
            
      float distance= sqrt(sq(EndX)+sq(EndY));
      if (Home_var==false)
         { if ((distance<286 && distance>=120 && EndX>0) || Move==0) 
              { Previous_EndX= EndX;
                Previous_EndY= EndY;
              }
           else 
              { EndX=Previous_EndX;
                EndY=Previous_EndY; 
              } 
           if (EndY<0 && EndX<120)
              {EndX=120;}    
           if ( Width<1800 && distance>=285)
              { Move_robot(1);}
         }         
     }   
}                     

void motion()
{  
   phi_1=    (atan2(EndY,EndX))*57.2957795;
   phi_2=    acos((sq(L1)+sq(EndX)+sq(EndY)-sq(L2)) / (2*L1*sqrt(sq(EndX)+sq(EndY))))*57.2957795;
   Theta_1=  phi_1+phi_2;
   Theta_2=  acos( (-sq(EndX)-sq(EndY)+sq(L1)+sq(L2)) / ((2*L1*L2))  )*57.2957795;

   if ( Theta_1<=180)
      { Theta_2=Theta_2-90;
         if ( Theta_2<-40)
            { Theta_2=-40;} }  
   if ( Theta_1>180)
      { Theta_1=phi_1-phi_2;
        Theta_2=(90-Theta_2+180); } 
   if ( Theta_1<0)
      { Theta_1=0; } 

   if (EndY<0)
      {orientation_angle=155;}
   Theta_3=orientation_angle-Theta_1-Theta_2;       
   if ( Theta_3<-20)  
      { Theta_3=-20; } 
   if ( Theta_3>210)  
      { Theta_3=210; }   

   if (Theta_turn>150)
      {Theta_turn=150;}
   if (Theta_turn<30)
      {Theta_turn=30;}
 
   Theta_2=map(Theta_2,-45,225,0,180);
   Theta_3=map(Theta_3,-45,225,0,180);
   myservo_base.write(Theta_1);  // 175 / 87/ 0                         
   myservo_mid.write(Theta_2);   // (180 @ 150), (0 @ 30) // map(a,-45,225,0,180);
   myservo_up.write(Theta_3);
   myservo_turn.write(Theta_turn); // between 50, 130  
   //myservo_grab.write(Theta_1);
   //myservo_cut.write(Theta_1);
   
}


float speed_map(float(val),float(x1),float(y1),float(x2),float(y2))
{ x= (x2-y2)/(x1-y1);
  y= x2-(x1*x); 
  return ((val*x)+y); 
} 

void Home()        // When EndX & EndY are within the usual workspace, then brings robot back to home position slowly. 
{ Move_robot(0);
  float Home_Y= speed_map(abs(130-EndY),150,0,2.2,1.0); 
  float Home_X= speed_map(abs(1-EndX),150,0,2.2,1.0); 
  if (EndY>130)
     {EndY=EndY-Home_Y;}
  if (EndY<130)
     {EndY=EndY+Home_Y;}  
  if (EndY>0)
     { if (EndX>1)
          {EndX=EndX-Home_X;}
       if (EndX<1)
          {EndX=EndX+Home_X;}    
     } 
} 

void grab()
{
  myservo_grab.write(90); // grab=90, open=110
}

void Open()
{
  myservo_grab.write(120); // grab=90, open=110
}

void Back_to_Home()  // When EndX & EndY are out of usual workspace, then brings robot back to home position slowly. 
{ if (Theta_1<180)
     { Theta_1+=0.022;} 
  if (Theta_1>180) 
     { Theta_1-=0.022;} //0.006 is when nothing is printing  // now 0.02
  if (Theta_2<3)
     { Theta_2+=0.022;} 
  if (Theta_2>3)
     { Theta_2-=0.022;}    
  if (Theta_3<59)
     { Theta_3+=0.013;} 
  if (Theta_3>59)
     { Theta_3-=0.013;}       
   myservo_base.write(Theta_1);  // 175 / 87/ 0                         
   myservo_mid.write(Theta_2);   // (180 @ 150), (0 @ 30) // map(a,-45,225,0,180);
   myservo_up.write(Theta_3);
   if (round(Theta_1)==180 && round(Theta_2)==3 && round(Theta_3)==59)
      {Step=1;
       EndX=1;
       EndY=130;
       wait=1;
      }
}

void place() // goes till thr trunk 
{ if (Theta_1<37)
     { Theta_1+=0.012;} //0.004 is when nothing is printing // now 0.015
  if (Theta_1>37)
     { Theta_1-=0.012;}    
  if (Theta_2<170)
     { Theta_2+=0.017;} 
  if (Theta_2>170)
     { Theta_2-=0.017;}    
  if (Theta_3<148)
     { Theta_3+=0.017;} 
  if (Theta_3>148)
     { Theta_3-=0.017;}    
   myservo_base.write(Theta_1);  // 175 / 87/ 0                         
   myservo_mid.write(Theta_2);   // (180 @ 150), (0 @ 30) // map(a,-45,225,0,180);
   myservo_up.write(Theta_3);
   if (round(Theta_1)==37 && round(Theta_2)==170 && round(Theta_3)==148)
      {  Open();}
}

void pick_and_place() // currently its only place_home
{ if (Serial.available()>0)
     { String str= Serial.readStringUntil('|');
       Serial_input= (str.toInt()); 
       if (Serial_input>1400 || Serial_input==0) // its width then 
          { Width=round((Width*9/10)+(Serial_input*1/10)); 
                if (Width<1650 || Serial_input==0)   // means failed to pickup the orange 
                   { wait_time=millis()-5000; 
                     Open(); } 
          }    
     }     
  
  long diff= millis()-wait_time; // gives ample time for tasks to complete 
  if ( diff<2000) // 2000
     { grab();
       motion();} 
  if ( diff>=2000 && diff<3400)  // 2000 and 5000
     { place(); } 
  if ( diff>=3400 && diff<4200)   // 5000 and 6000
     { Open();
       myservo_grab.write(120);
     }   
  if ( diff>=4200 && diff<6500)    // 6000 and 10000 
     { Back_to_Home(); }         
}

void look_out() 
{ if (Serial.available()>0)
     { String str= Serial.readStringUntil('|');
       Serial_input= (str.toInt()); 
       if (Serial_input>1400) // its width then 
       { Step=1; } 
     } 
  float Speed_YY= 0.025;
  float Speed_XX= 0.025;
  if (look_out_var==0)
     {if (EndY>130)
         {EndY=EndY-Speed_YY;}
      if (EndY<130)
         {EndY=EndY+Speed_YY;}  
      if (EndX>100)
         {EndX=EndX-Speed_XX;}
      if (EndX<100)
         {EndX=EndX+Speed_XX;} 
      if (Theta_turn>50)
         {Theta_turn=Theta_turn-0.005;}
      if (Theta_turn<50)
         {Theta_turn=Theta_turn+0.005;}
     }    
  if (round(EndY)==130 && round(EndX)==100 && round(Theta_turn)==50)
     {look_out_var=1; }
  if (look_out_var==1)
     { if (Theta_turn>130)
          {Theta_turn=Theta_turn-0.005;}
       if (Theta_turn<130)
          {Theta_turn=Theta_turn+0.005;}
     }
  if (round(EndY)==130 && round(EndX)==100 && round(Theta_turn)==130)
     {look_out_var=2; }
  if (look_out_var==2)
     { if (EndY>-20)
          {EndY=EndY-Speed_YY;}
       if (EndY<-20)
          {EndY=EndY+Speed_YY;}  
       if (EndX>130)
          {EndX=EndX-Speed_XX;}
       if (EndX<130)
          {EndX=EndX+Speed_XX;} 
     }  
   if (round(EndY)==(-20) && round(EndX)==130 && round(Theta_turn)==130)
     {look_out_var=3; }  

   if (look_out_var==3)
     { if (Theta_turn>50)
          {Theta_turn=Theta_turn-0.005;}
       if (Theta_turn<50)
          {Theta_turn=Theta_turn+0.005;} 
     }    
   if (look_out_var==3 && (round(Theta_turn)==50))
      { look_out_var=0; }      
  motion();
}

int Move_robot(int x)
{ if (x==0)               // Stop 
     { myservo_1.write(90);
       myservo_2.write(90);
       myservo_3.write(90);
       myservo_4.write(90);
     }
  if (x==1)               // Forward 
     { myservo_1.write(180);
       myservo_2.write(0);
       myservo_3.write(0);
       myservo_4.write(180);
     }   
  if (x==2)               // Reverse 
     { myservo_1.write(0);
       myservo_2.write(180);
       myservo_3.write(180);
       myservo_4.write(0);
     }
  if (x==3)               // Left 
     { myservo_1.write(180);
       myservo_2.write(180);
       myservo_3.write(180);
       myservo_4.write(180);
     }
  if (x==4)               // Right 
     { myservo_1.write(0);
       myservo_2.write(0);
       myservo_3.write(0);
       myservo_4.write(0);
     }    
}
