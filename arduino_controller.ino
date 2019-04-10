/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    Me_Megapi_encoder_pid_pos.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/07/14
 * @brief   Description: this file is sample code for Megapi encoder motor device.
 *
 * Function List:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);
 *    3. void MeEncoderOnBoard::pulsePosPlus(void);
 *    4. void MeEncoderOnBoard::pulsePosMinus(void);
 *    5. void MeEncoderOnBoard::setMotorPwm(int pwm);
 *    6. double MeEncoderOnBoard::getCurrentSpeed(void);
 *    7. void MeEncoderOnBoard::setSpeedPid(float p,float i,float d);
 *    8. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
 *    7. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
 *    8. void MeEncoderOnBoard::setPulse(int16_t pulseValue);
 *    9. void MeEncoderOnBoard::setRatio(int16_t RatioValue);
 *    10. void MeEncoderOnBoard::moveTo(long position,float speed,int16_t extId,cb callback);
 *    11. void MeEncoderOnBoard::loop(void);
 *    12. long MeEncoderOnBoard::getCurPos(void);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/07/14    1.0.0          build the new
 * </pre>
 */

#include <MeMegaPi.h>

MeMegaPiDCMotor motor1(PORT4B);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void isr_process_encoder3(void)
{
  if(digitalRead(Encoder_3.getPortB()) == 0)
  {
    Encoder_3.pulsePosMinus();
  }
  else
  {
    Encoder_3.pulsePosPlus();
  }
}

void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);
  Serial3.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(7);
  Encoder_2.setPulse(7);
  Encoder_3.setPulse(7);
  Encoder_1.setRatio(26.9);
  Encoder_2.setRatio(26.9);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_3.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
  Encoder_3.setSpeedPid(0.18,0,0);
}

long arm_time = 0;
long arm_value = 0;
long last_print = 0;
void loop()
{
  if (arm_value == 1 && arm_time == 0) {
    motor1.run(-200);
    arm_time = millis();
  } else if (arm_value == 1 && millis() - arm_time > 250) {
    motor1.stop();
    arm_time = 0;
    arm_value = 3;
  } else if (arm_value == 2 && arm_time == 0) {
    motor1.run(200);
    arm_time = millis();
  } else if (arm_value == 2 && millis() - arm_time > 250) {
    motor1.stop();
    arm_time = 0;
    arm_value = 3;
  }
    
  if(Serial3.available())
  {
    char a = Serial3.read();
    switch(a)
    {
      case ' ':
      Encoder_1.setSpeed(0);
      Encoder_2.setSpeed(0);
      Encoder_3.setSpeed(0);
      motor1.stop();
      break;
      case 'a':
      Encoder_1.move(100,400);
      Encoder_2.move(100,400);
      break;
      case 'd':
      Encoder_1.move(-100,400);
      Encoder_2.move(-100,400);
      break;
      case 'w':
      Encoder_1.move(600,600);
      Encoder_2.move(-600,600);
      break;
      case 's':
      Encoder_1.move(-600,600);
      Encoder_2.move(600,600);
      break;
      case '4':
      Encoder_1.moveTo(-360,50);
      Encoder_2.moveTo(360);
      break;
      case '5':
      Encoder_1.moveTo(-1800,50);
      Encoder_2.moveTo(1800,50);
      break;
      case '6':
      Encoder_1.moveTo(-3600,50);
      Encoder_2.moveTo(3600,50);
      break; 
      case 'k':
      Encoder_3.move(150,200);
      break;
      case 'i':
      Encoder_3.move(-150,200);
      break;
      case 'o':
      arm_value = 1;
      break;
      case 'p':
      arm_value = 2;
      break;
      default:
      break;
    }
  }
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();

  if (millis() - last_print > 1000) {
    last_print = millis();
    Serial3.print("Spped 1:");
    Serial3.print(Encoder_1.getCurrentSpeed());
    Serial3.print(" ,Spped 2:");
    Serial3.print(Encoder_2.getCurrentSpeed());
    Serial3.print(" ,Spped 3:");
    Serial3.print(Encoder_3.getCurrentSpeed());
    Serial3.print(" ,CurPos 1:");
    Serial3.print(Encoder_1.getCurPos());
    Serial3.print(" ,CurPos 2:");
    Serial3.println(Encoder_2.getCurPos());
    Serial3.print(" ,CurPos 3:");
    Serial3.println(Encoder_3.getCurPos());
  }
}