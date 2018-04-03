void GetPyramid()
{
  
  //Move Linear Acuator forward
  LAPyramid.attach(13);
  LAPyramid.write(180);
  delay(1000);
  

  //Drop down pivot arm
  PivotMotor.attach(12);
  PivotMotor.write(45);
  delay(1000);


  //retract linear actuator
  LAPyramid.write(0);


}

