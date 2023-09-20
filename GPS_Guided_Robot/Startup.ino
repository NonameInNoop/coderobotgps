
void Startup()
{
     myservo.detach(); 
     Serial.println("Pause for Startup... ");
             
     for (int i=5; i >= 1; i--)                     
      {         
        Serial1.print("Pause for Startup... "); 
        Serial1.print(i);
        delay(1000);                             
      }    
    
  Serial1.println("Searching for Satellites "); 
  Serial.println("Searching for Satellites "); 
      
  while (Number_of_SATS <= 4)                      
  {                                 
    getGPS();                                       
    Number_of_SATS = (int)(gps.satellites.value());        
    bluetooth();                                      
  }    
  setWaypoint();                                 
  wpCount = 0;                                        
  ac = 0;                                           
  
  Serial1.print(Number_of_SATS);
  Serial1.print(" Satellites Acquired");    
}    
 

  
    
  



   
 
  
