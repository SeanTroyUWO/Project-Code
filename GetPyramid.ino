void GetPyramid()
{
  
  //Move Linear Acuator forward
  LAPyramid.attach(12);
  LAPyramid.write(180);
  delay(1000);
  

  //Drop down pivot arm
  PivotMotor.attach(13);
  PivotMotor.write(45);
  delay(1000);


  //retract linear actuator
  LAPyramid.write(0);


}

