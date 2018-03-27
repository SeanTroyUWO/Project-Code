void FollowWall()
{
  ArmMotor.detach();
  Ping();
//  PingFront();

  if (Corner == false)
  {

    //Side Sensor
    if ((EchoTimeSide / 58.0) > 3)                //if side sensor is too far away from wall...turn right
    {
      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1650);
      Serial.println(EchoTimeSide / 58.0);
      Serial.println("RIGHT");

    }

    //Side Sensor
    if ((EchoTimeSide / 58.0) < 1)                  //if side sensor is too close to wall...turn left
    {
      servo_LeftMotor.writeMicroseconds(1650);
      servo_RightMotor.writeMicroseconds(1600);
      Serial.println(EchoTimeSide / 58.0);
      Serial.println("LEFT");
    }

    // Front Sensor
    /*else if ((EchoTimeFront / 58.0) < 3)        //If front sensor detects wall...stop
    {
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
     // Serial.println(EchoTimeFront / 58.0);
      Serial.println("STOP");

      Corner = true;

    }*/

    else                                             //go straight
    {
      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1600);
      Serial.println(EchoTimeSide / 58.0);
      Serial.println("STRAIGHT");

    }

  }

/*
  if (Corner == true)
  {

    if ((EchoTimeFront / 58.0) < 5)
    {
      servo_LeftMotor.writeMicroseconds(1400);
      servo_RightMotor.writeMicroseconds(1400);
      Serial.println(EchoTimeFront / 58.0);
      Serial.println("BACKWARDS");


    }

    if ((EchoTimeSide / 58.0) < 3)                    //*******NEED TO FIX********
    {
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
      Serial.println(EchoTimeFront / 58.0);
      Serial.println("TURN COMPLETE");

      Corner = false;

    }

    else
    {
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1600);
      Serial.println(EchoTimeSide / 58.0);
      Serial.println("TURN");

    }






  }
  */







}

