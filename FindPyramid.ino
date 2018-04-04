void FindPyramid()
{
  int lightSensor_int;

  //Go in cirlces until it detects the pyramid
  if (PyramidFound == false){
    do{
      lightSensor_int = analogRead(IRFlag);
      Serial.println(lightSensor_int);

      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1500);

    } while (lightSensor_int < 80);

    //When pyramid is found, robot stops, PyramidFound flag is raised
    if (lightSensor_int > 80){
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);

      PyramidFound = true;
    }
    
    else{
      PyramidFound = false;
    }
  }
}


