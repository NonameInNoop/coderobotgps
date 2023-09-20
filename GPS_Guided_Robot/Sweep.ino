

void sweep()                          
{ 
  myservo.attach(51); 
  StopCar();
  Forward_Meter();
  StopCar();
    
  for(pos = 60; pos <= 120; pos += 1)  
  {                                   
    myservo.write(pos);               
    delay(15);                        
  } 
  for(pos = 120; pos>=60; pos-=1)      
  {                                
    myservo.write(pos);               
    delay(15);                    
  } 

    myservo.write(90);                
    delay(15);                        
   myservo.detach();                  
} 
