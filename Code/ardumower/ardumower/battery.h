// battery code

// check battery voltage and decide what to do
void Robot::checkBattery(){
  if (millis() < nextTimeCheckBattery) return;
	nextTimeCheckBattery = millis() + 1000;  
  if (batVoltage < 4.0){      
    // ROS raise error battery
    //Console.println(F("BATTERY NOT FOUND - PLEASE SWITCH ON BATTERY!")); 
  }
    
  if (batMonitor){
    if ((batVoltage < batSwitchOffIfBelow) && (idleTimeSec != BATTERY_SW_OFF)) {      
			/* Console.print(F("Battery warning: triggered batSwitchOffIfBelow "));
			Console.print(batVoltage);
			Console.print(F("<"));
			 Console.println(batSwitchOffIfBelow); */
			 // ROS raise event battery
      addErrorCounter(ERR_BATTERY);      
			delay(2000); // avois corrupting EEPROM while this is also called when power is turned OFF
			beep(2, true);      
			loadSaveErrorCounters(false); // saves error counters
      loadSaveRobotStats(false);    // saves robot stats
      idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
      Console.println(F("BATTERY switching OFF"));

		// ROS raise shutdown, RPI must be shutdown
/*       if (rmcsUse)  // tell Raspberry PI to shutdown
      {
        rmcsSendOff(Console);
      } */
      setActuator(ACT_BATTERY_SW, 0);  // switch off battery                     
    }

  
	  // check if idle and robot battery can be switched off  
		if ( (stateCurr == STATE_OFF) || (stateCurr == STATE_ERROR) ) {      
			if (idleTimeSec != BATTERY_SW_OFF){ // battery already switched off?
				idleTimeSec ++; // add one second idle time
				if ((batSwitchOffIfIdle != 0) && (idleTimeSec > batSwitchOffIfIdle * 60)) {        
					//Console.println(F("triggered batSwitchOffIfIdle"));      
					// ROS raise event bat switch off
					beep(1, true);      
					loadSaveErrorCounters(false); // saves error counters
					loadSaveRobotStats(false);    // saves robot stats
					idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
					//Console.println(F("BATTERY switching OFF"));

                    /* if(rmcsUse)  // Tell Raspberry PI to shutdown
                    {
                       rmcsSendOff(Console);
                    } */
					// ROS raise shutdown
					setActuator(ACT_BATTERY_SW, 0);  // switch off battery               
				}
			}
		} else {
			resetIdleTime();          
		}
	}
}
