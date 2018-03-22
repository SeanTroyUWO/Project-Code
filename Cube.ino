void Cube()
{

  //when limit switch detects cube...stop robot
  if (LimitSwitch == HIGH)
  {

    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    Serial.println("Cube Detected");

    CubeDetected == true;


  }

  //when cube is detectred and robot is stopped...
  if (CubeDetected == true)
  {

    //get cube

    //when we have cube
    StageCounter++;

  }








}

