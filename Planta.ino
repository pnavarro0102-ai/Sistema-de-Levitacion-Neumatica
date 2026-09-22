#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

TFLI2C tflI2C;
LiquidCrystal_I2C lcd(0x27, 20, 4);

#define D12_PIN        12
#define D13_PIN        13
#define PWM_PIN         9       
#define SETPOINT_PIN   A3
#define KP             A2
#define KI             A1
#define KD             A0

const float ADC_MAX = 1023.0f;
const unsigned long INTERVALO_MUESTREO_MS = 10;

double ref_cm = 0.0;
float kp_r = 0.0;
float ki_r = 0.0;
float kd_r = 0.0;

int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR;
int     Distancia;

double uk=0, xk=0, ek=0;
double ukm1=0, xkm1=0, ukm2=0, xkm2=0;
double ekm1=0, ekm2=0;

int i=0;
float Ts=0.01;
float kp=3.6;
float ti=kp/6;
float td=0.09/kp;
double A, B, C;
int pwm;
float reference;
float ref_kp, ref_ki, ref_kd;
int mode;
int max_pwm;

void printLine(uint8_t row, const String &text) {
  lcd.setCursor(0, row);
  lcd.print("                    ");
  lcd.setCursor(0, row);
  lcd.print(text);
}

void setup() {
  A = kp*(1+(Ts/(2*ti))+(td/Ts));
  B = -kp*(1-(Ts/(2*ti))+(2*td/Ts));
  C = kp*td/Ts;

  Serial.begin(115200);
  Wire.begin();

  pinMode(D12_PIN, INPUT);
  pinMode(D13_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  printLine(0, "  ***BIENVENIDO*** ");
  printLine(1, "     Levitador");
  printLine(2, "     Neumatico");
  printLine(3, "               -TEC");
  delay(4000);

  lcd.clear();
  printLine(0, "    *Hecho por: ");
  printLine(1, "-Jeremy Soto");
  printLine(2, "-Pablo Navarro");
  printLine(3, "-Vladimir Gonzalez");
  delay(4000);
}

void loop() {
  mode    = digitalRead(D13_PIN);
  max_pwm = digitalRead(D12_PIN);

  if (max_pwm == 1) {
    analogWrite(PWM_PIN, 255);
    printLine(0, "                    ");
    printLine(1, "-MAX_PWM");
    printLine(2, "                    ");
    printLine(3, "                    ");
  } else {
    if (mode == 1) {
      float Ts=0.01;
      float kp=0.3945*7;
      float ti=kp/(0.1268*40);
      float td=0.1/kp;

      A = kp*(1+(Ts/(2*ti))+(td/Ts));
      B = -kp*(1-(Ts/(2*ti))+(2*td/Ts));
      C = kp*td/Ts;

      reference = analogRead(SETPOINT_PIN);
      ref_cm = map(reference, 0, 1024, 0, 81);

      if (tflI2C.getData(tfDist, tfAddr)) {
        if (tfDist < 50) Distancia = 85 - tfDist;
        else             Distancia = 87 - tfDist;
    
        xk = Distancia;
        ek = ref_cm - xk;
        uk = ukm1 + (A*ek) + (B*ekm1) + (C*ekm2);

        if (uk >= 255) uk = 255;
        if (uk <= 0)   uk = 0;
        pwm = floor(uk);
        analogWrite(PWM_PIN, pwm);

        printLine(0, "  -Modo automatico  ");
        printLine(1, "                 ");
        printLine(2, "    Sensor: " + String(Distancia) + " cm");
        printLine(3, "    Ref : " + String(ref_cm, 0) + " cm");
        delay(INTERVALO_MUESTREO_MS);

        if (i>=2) {
          xkm2=xkm1;
          ukm2=ukm1;
          ekm2=ekm1;
        }
        xkm1 = xk;
        ukm1 = uk;
        ekm1 = ek;
        i += 1;
      }
    }

    if (mode == 0) {
      int a_kp = analogRead(KP);
      int a_ki = analogRead(KI);
      int a_kd = analogRead(KD);

      ref_cm = analogRead(SETPOINT_PIN) * (81.0f / 1023.0f);

      kp_r = a_kp * (10.0f   / 1023.0f);
      ki_r = a_ki * (15.0f   / 1023.0f);
      kd_r = a_kd * (0.50f  / 1023.0f);

      if (kp_r < 0.05f) kp_r = 0.05f;
      if (ki_r < 0.05f) ki_r = 0.05f;

      float Ts = 0.01f;
      float kp = kp_r;
      float ti = kp_r / ki_r;
      float td = kd_r/kp_r;

      if (ti < 0.15f) ti = 0.15f;
      if (ti > 2.00f) ti = 2.00f;
      if (td < 0.0f)  td = 0.0f;
      if (td > 0.08f) td = 0.08f;

      A = kp * (1 + (Ts/(2*ti)) + (td/Ts));
      B = -kp * (1 - (Ts/(2*ti)) + (2*td/Ts));
      C = kp * td / Ts;

      if (isnan(A) || isnan(B) || isnan(C) ||
          isinf(A) || isinf(B) || isinf(C)) {
        A = 0; B = 0; C = 0;
      }

      if (tflI2C.getData(tfDist, tfAddr)) {
        Distancia = (tfDist < 50) ? (85 - tfDist) : (87 - tfDist);

        xk = Distancia;
        ek = ref_cm - xk;

        uk = ukm1 + (A*ek) + (B*ekm1) + (C*ekm2);

        if (isnan(uk) || isinf(uk)) uk = ukm1;
        if (uk > 255) uk = 255;
        if (uk < 0)   uk = 0;

        pwm = (int)floor(uk + 0.5);
        analogWrite(PWM_PIN, pwm);

        ekm2 = ekm1; ekm1 = ek;
        ukm2 = ukm1; ukm1 = uk;
        xkm2 = xkm1; xkm1 = xk;

        printLine(0, "    -Modo manual  ");
        printLine(1, "Ref:" + String(ref_cm, 0) + "cm" +
                      " Sensor:" + String(Distancia) + "cm");
        printLine(2, "KP: " + String(kp_r,2) + "   KI: " + String(ki_r,2));
        printLine(3, "     KD: " + String(kd_r,3));

        delay(INTERVALO_MUESTREO_MS);
      }
    }
  }
}
