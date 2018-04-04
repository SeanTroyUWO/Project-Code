void GetPyramid()
{
  
  //Move Linear Acuator forward
  LAPyramid.attach(13);
  LAPyramid.write(140);
  delay(5000);
  Serial.println("140");

  //Drop down pivot arm
  //PivotMotor.attach(12);
  //PivotMotor.write(45);
  LAPyramid.write(0);
  delay(5000);


  //retract linear actuator
  //LAPyramid.write(0);


}

