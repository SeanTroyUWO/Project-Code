void FinalStep()
{
  //Put arm into cube-dropping position
  ArmMotor.attach(11);
  ArmMotor.write(0);
  ArmMotor.detach();


  //Drop Cube
  CubeMotor.write(180);
  Serial.println("Cube Dropped");

  delay(2000);


  //Drop Pyramid on cube
  LAPyramid.write(0);



  Serial.println("CONGRATULATIONS!!!!!!!");



}

