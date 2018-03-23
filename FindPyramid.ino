void FindPyramid()
{
  int lightSensor_int;


  //Go in cirlces until it detects the pyramid
  if (PyramidFound == false)
  {
    do
    {
      lightSensor_int = IRSensor.read();
      Serial.print(lightSensor_int);
      Serial.print("\n");

      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1500);

    } while ((lightSensor_int == -1) || (lightSensor_int != 65));


    //When pyramid is found, robot stops, PyramidFound flag is raised
    if (lightSensor_int == 65)
    {
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);

      PyramidFound = true;

    }
    else
    {
      PyramidFound = false;
    }

  }



  //When found, drive towards the pyramid
  if (PyramidFound == true)
  {
    servo_LeftMotor.writeMicroseconds(1600);
    servo_RightMotor.writeMicroseconds(1600);
  }



}

