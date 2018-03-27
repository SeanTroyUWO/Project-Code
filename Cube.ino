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

    CubeMotor.write(90);
    Serial.println("Cube has been grabbed");

    CubeGrabbed = true;


  }


  if (CubeGrabbed == true)
  {

    ArmMotor.write(180);
    Serial.println("Ready to drop cube");

    CubeMotor.write(0);
    Serial.println("Cube Dropped");

    //Next Stage
    StageCounter = 2;

  }




}

