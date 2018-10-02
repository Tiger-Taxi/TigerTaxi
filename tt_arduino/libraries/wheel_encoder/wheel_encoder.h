#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H



void setup_Encoders();
long read_encoder(int encoder);
void ClearEncoderCount();
void read_encoders();
long GetLeftEncoder();
long GetRightEncoder();

#endif
