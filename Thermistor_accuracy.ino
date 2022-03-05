//
// ADC accuracy improvement function
//
int16_t ADCvalue;
void setup(){
  Serial.begin(9600);
}

void loop() {
  Serial.print(analogRead(A0));
  Serial.print("\t");
  Serial.println(ReadVoltage(A0));
  Serial.print("Temp");
  ADCvalue = analogRead(A0);
  Thermistor(ADCvalue);
}

double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required

void Thermistor(int16_t ADCvalue)
{
  double T, Temp;
  double T0 = 301.15;  // 273.15 + 28 (room temperature) 室溫換成絕對溫度
  double lnR;
  int16_t R;          // Thermistor resistence 
  int16_t R0 = 8805;  // calibrated by reading R at room temperature (=28 degree Celsius )
  int16_t B  = 3950;
  int16_t Pullup = 9930; // 10K ohm
  
  // R / (Pullup + R)   = adc / 4096
  R = (Pullup * ADCvalue) / (4096 - ADCvalue);
  
  // B = (log(R) - log(R0)) / (1/T - 1/T0) 
  T = 1 / (1/T0 + (log(R)-log(R0)) / B );
  Temp = T - 273.15;  
    
  Serial.println(Temp);
}
/* ADC readings v voltage
 *  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */
