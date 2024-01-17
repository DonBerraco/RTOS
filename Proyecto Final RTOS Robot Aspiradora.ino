// Incluye las bibliotecas necesarias
#include <DHT.h>
#include <Robojax_L298N_DC_motor.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <Wire.h>
// Define una estructura para el manejo de banderas
typedef struct _vFlag
{
  uint8_t BTFlag = 0;
  uint8_t L298NFlag = 0;
  uint8_t HCSR04Flag = 1;
  uint8_t LEDFlag = 1;
  uint8_t ServoFlag = 0;
  uint8_t initial_Flag = 0;
  uint8_t FunctionFlag = 0;
  uint8_t back_light_Flag = 0;
  uint8_t front_light_Flag = 0;
} vFlag;
vFlag *flag_Ptr;
vFlag flag;
// Bluetooth
BluetoothSerial SerialBT;
// ServoMotor
Servo myservo;  
#define servoPin 13

// LED
#define LED_BUILTIN 2

// L298
// Configuración del controlador de motor L298N
// Ajustes y Configuración: MOTOR 1
#define IN1 16
#define IN2 17
#define ENA 4 /
#define CHA 0
// Ajustes y Configuración: MOTOR 2
#define IN3 18
#define IN4 19
#define ENB 5 
#define CHB 1
const int CCW = 2; 
const int CW  = 1; 

#define motor1 1 // No cambiar
#define motor2 2 // No cambiar
// DHT11
// Configuración del sensor DHT
#define DHTPIN 23
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

Robojax_L298N_DC_motor motor(IN1, IN2, ENA, IN2, IN3, ENB, true);
//------Para control de bluetooth -----------------------
// Configuración de las luces
#define back_light 21
#define front_light 22
// HCSR04
// Configuración del sensor ultrasónico HCSR04
#define TRIGPIN_PIN 12
#define ECHO_PIN 14 
long duration;
unsigned long currentMillis = 0;
// Variables Globales
#define MAX_DISTANCE 200
#define MAX_SPEED 200 // Establece la velocidad de los motores de DC
int speedSet = 0;
int distance = 15;
int distanceR = 10;
int distanceL = 10;
// UART
// Configuración de UART
#define LINE_BUFFER_LENGTH 64
typedef struct _vUart
{
  char c;
  int lineIndex = 0;
  int line1Index = 0;
  int BTlineIndex = 0;
  bool lineIsComment; 
  bool lineSemiColon;

  char line[128];

  char BTline[20];

  String inputString;
  String BTinputString;
  String S1inputString;
  int V[16];
  char ctemp[30];
  char I2C_Data[80];
  int DC_Spped = 50;
  float Voltage[16];
  int Buffer[128];
  int StartCnt = 0;
  int ReadCnt = 0;
  int sensorValue = 0;
} vUart;
vUart *Uart_Ptr;
vUart Uart;

TaskHandle_t huart;
TaskHandle_t hfunction;
// Prototipos de funciones
void vUARTTask(void *pvParameters);
void vFunctionTask(void *pvParameters);
// Función para inicializar tareas
void initial()
{
  Serial.println(F("Create Task"));
  // Crea la tarea UART
  xTaskCreatePinnedToCore(
    vUARTTask, 
    "UARTTask",
    1024,
    NULL, 
    3,
    &huart,
    0);
  // Crea la tarea de función
  xTaskCreatePinnedToCore(
    vFunctionTask, 
    "FunctionTask", 
    1024 ,
    NULL,
    1,
    &hfunction, 
    1);
}
// Función para leer la temperatura del sensor DHT
void leerTemperatura() {
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Error al leer la temperatura desde DHT11");
    return;
  }

  SerialBT.print("Temperatura: ");
  SerialBT.print(temperature);
  SerialBT.println(" °C");
}
// Funciones de control del motor
void Forward() 
{  
  
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 100);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 100);
}

void Reverse(){
  //motors.rotate(motor2, 70, CW);
  //motors.rotate(motor1, 70, CW);
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA, 100);
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
  analogWrite(ENB, 100);
}
void Left()
{
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 100);
}
void Right()
{
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 100);
}
void Stop() 
{
  motor.brake(1);
  motor.brake(2);
  //myservo.detach(); 
}
// Configuración inicial de Arduino
void setup() 
{
  Serial.begin(9600);
  Serial.println(F("init"));
  initial();
  SerialBT.begin("BT_L298N");
  dht.begin();
  myservo.setPeriodHertz(50);    
  myservo.attach(servoPin, 500, 2400); 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIGPIN_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(back_light, OUTPUT);
  pinMode(front_light, OUTPUT);
  motor.begin();
  myservo.write(90);
}

void loop() 
{
  leerTemperatura(); // Lee la temperatura del sensor DHT y la envía por Bluetooth
  Serial.print(F("Main at core:"));
  Serial.println(xPortGetCoreID()); // Imprime el núcleo actual del ESP32
  while(1) // Bucle infinito
  {
    if(flag.HCSR04Flag==1) // Verifica si la bandera HCSR04Flag es 1 (activado)
    {
      if(distance<=20) // Si la distancia medida por el sensor ultrasónico es menor o igual a 20 unidades
      {
        Stop(); // Detiene el robot
        Reverse(); // Retrocede el robot
        delay(750); // Espera 750 milisegundos
        Stop(); // Detiene el robot
        delay(250); // Espera 250 milisegundos
        flag.HCSR04Flag=2; // Cambia el estado de la bandera HCSR04Flag a 2
        delay(500); // Espera 500 milisegundos
        flag.HCSR04Flag=3; // Cambia el estado de la bandera HCSR04Flag a 3
        delay(500); // Espera 500 milisegundos
        flag.HCSR04Flag=1; // Vuelve al estado 1 de la bandera HCSR04Flag

        if ((distanceR >= distanceL) ) 
         {
          Left();
          delay(1500);
          Stop();
          delay(200);
          flag.HCSR04Flag=1;
        } 
        else 
        { 
          Right();
          delay(1500);
          Stop();
          delay(200);
          flag.HCSR04Flag=1;
        }
        myservo.write(90);
        delay(500);
      } 
      else 
      {
        flag.HCSR04Flag=1;
        Forward();
        delay(100);
        Stop();
        delay(30);
      }      
    }
    vTaskDelay(1);
  }
}
//----------------------------------------
void processCommand(char *data)
{
  int len, xlen, ylen, zlen, alen;
  int tempDIO;
  String stemp;

  len = Uart.inputString.length();
  //---------------------------------------
  if (strstr(data, "VER") != NULL)
  {
    Serial.println(F("ESP32_20230710"));
  }
  //-------------- HCSR04 --------------------
  if (strstr(data, "HCSR04_ON") != NULL)
  {
    flag.HCSR04Flag = 1;
    Serial.println(F("HCSR04_ON"));
  }
  if (strstr(data, "HCSR04_OFF") != NULL)
  {
    flag.HCSR04Flag = 0;
    Serial.println(F("HCSR04_OFF"));
  }  
  //----------------L298N-----------
  if (strstr(data, "F") != NULL)
  {
    Serial.println(F("Forward"));
    Forward();
    //forward();
    
  }
  if (strstr(data, "L") != NULL)
  {
    Serial.println(F("Left"));
    Left();
  }
  if (strstr(data, "R") != NULL)
  {
    Serial.println(F("Right"));
    Right();
  }
  if (strstr(data, "B") != NULL)
  {
    Serial.println(F("Reverse"));
    Reverse();
    //backward();
  }
  if (strstr(data, "S") != NULL)
  {
    Serial.println(F("Stop"));
    Stop();
  }
  //-------------- Servo --------------------
  if (strstr(data, "SERVO_5")!= NULL)
  {
    Serial.println(F("SERVO_5"));
    myservo.write(5);
    //myservo.detach();
  }
  if (strstr(data, "SERVO_10")!= NULL)
  {
    Serial.println(F("SERVO_10"));
    myservo.write(10);
  }
  if (strstr(data, "SERVO_20")!= NULL)
  {
    Serial.println(F("SERVO_20"));
    myservo.write(20);
  }
  if (strstr(data, "SERVO_30")!= NULL)
  {
    Serial.println(F("SERVO_30"));
    myservo.write(30);
  }
  if (strstr(data, "SERVO_50")!= NULL)
  {
    Serial.println(F("SERVO_50"));
    myservo.write(50);
  }
  if (strstr(data, "SERVO_80")!= NULL)
  {
    Serial.println(F("SERVO_80"));
    myservo.write(80);
  }
  if (strstr(data, "SERVO_90")!= NULL)
  {
    Serial.println(F("SERVO_90"));
    myservo.write(90);
  }
  if (strstr(data, "SERVO_100")!= NULL)
  {
    Serial.println(F("SERVO_100"));
    myservo.write(100);
  }
  if (strstr(data, "SERVO_120")!= NULL)
  {
    Serial.println(F("SERVO_120"));
    myservo.write(120);
  }
  if (strstr(data, "SERVO_140")!= NULL)
  {
    Serial.println(F("SERVO_140"));
    myservo.write(140);
  }
  if (strstr(data, "SERVO_150")!= NULL)
  {
    Serial.println(F("SERVO_150"));
    myservo.write(150);
  }
}
//-----------------------------------------
//-------------------BT-----------------
void BTprocessCommand(String data)
{
  if (data =="FS")
  {
    Serial.println(F("Atrás"));
    Forward();
  }
  if (data == "LS")
  {
    Serial.println(F("Izquierda"));
    Left();
  }
  if (data == "RS")
  {
    Serial.println(F("Derecha"));
    Right();
  }
  if (data == "BS")
  {
    Serial.println(F("Reversa"));
    Reverse();
  }
  if (data == "S")
  {
    Serial.println(F("paro"));
    Stop();
  }
  if (data == "X")
  {
    flag.HCSR04Flag=0;
    Serial.println(F("Paro"));
    Stop();
    flag.back_light_Flag=2;
    flag.HCSR04Flag=0;
  }
  if (data == "x")
  {
    Serial.println(F("Paro"));
    Stop();
    flag.back_light_Flag=0;
    flag.HCSR04Flag=1;
  }
  if (data == "FGFS")
  {
    //LF
    motor.rotate(motor1, 60, CCW);
    motor.rotate(motor2, 100, CCW);
  }
  if (data == "FIFS")
  {
    //RF
    motor.rotate(motor1, 100, CCW);
    motor.rotate(motor2, 60, CCW);
  }
  if (data == "BHBS")
  {
    //LB
    motor.rotate(motor1, 60, CW);
    motor.rotate(motor2, 100, CW);
  }
  if (data == "BJBS")
  {
    //RB
    motor.rotate(motor1, 100, CW);
    motor.rotate(motor2, 60, CW);
  }
  if (data == "U")
  {
    //backlight
    digitalWrite(back_light, HIGH);  
    flag.back_light_Flag=1;
    //Serial.println(F("light"));
  }
  if (data == "u")
  {
    //backlight
    digitalWrite(back_light, LOW);
    flag.back_light_Flag=0;
    //Serial.println(F("lightoff"));
  }
  if (data == "W")
  {
    digitalWrite(front_light, HIGH);  
    flag.front_light_Flag=1;
    //Serial.println(F("light"));
  }
  if (data == "w")
  {
    digitalWrite(front_light, LOW);
    flag.front_light_Flag=0;
    //Serial.println(F("lightoff"));
  }
}


//-------------------------------------------
void vUARTTask(void *pvParameters)
{
  (void)pvParameters;

  Serial.print(F("UARTTask at core:"));
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    while (Serial.available() > 0)
    {
      Uart.c = Serial.read();
  
      if ((Uart.c == '\n') || (Uart.c == '\r'))
      { 
        if (Uart.lineIndex > 0)
        { 
          Uart.line[Uart.lineIndex] = '\0'; 
          //Serial.println( F("Debug") );
          //Serial.println( Uart.inputString );
          processCommand(Uart.line); 
          Uart.lineIndex = 0;
          Uart.inputString = "";
        }
        else
        {
          
        }
        Uart.lineIsComment = false;
        Uart.lineSemiColon = false;
        Serial.println(F("ok>"));
      }
      else
      {
        //Serial.println( c );
        if ((Uart.lineIsComment) || (Uart.lineSemiColon))
        {
          if (Uart.c == ')')
            Uart.lineIsComment = false; 
        }
        else
        {
          if (Uart.c == '/')
          { 
          }
          else if (Uart.c == '~')
          { 
            Uart.lineIsComment = true;
          }
          else if (Uart.c == ';')
          {
            Uart.lineSemiColon = true;
          }
          else if (Uart.lineIndex >= LINE_BUFFER_LENGTH - 1)
          {
            Serial.println("ERROR - lineBuffer overflow");
            Uart.lineIsComment = false;
            Uart.lineSemiColon = false;
          }
          else if (Uart.c >= 'a' && Uart.c <= 'z')
          { // Upcase lowercase
            Uart.line[Uart.lineIndex] = Uart.c - 'a' + 'A';
            Uart.lineIndex = Uart.lineIndex + 1;
            Uart.inputString += (char)(Uart.c - 'a' + 'A');
          }
          else
          {
            Uart.line[Uart.lineIndex] = Uart.c;
            Uart.lineIndex = Uart.lineIndex + 1;
            Uart.inputString += Uart.c;
          }
        }
      }
    } //while (Serial.available() > 0)
    while (SerialBT.available())
    {
      flag.L298NFlag=1;
      String BTdata = SerialBT.readString();
      Stop();
      Serial.println(BTdata);
      BTprocessCommand(BTdata); // do something with the command
    }//while (BT.available())
    vTaskDelay(1);
  }
}
void vFunctionTask(void *pvParameters)
{
  (void)pvParameters;

  Serial.print(F("FunctionTask at core:"));
  Serial.println(xPortGetCoreID());
  for (;;) // A Task shall never return or exit.
  {
    if(flag.HCSR04Flag==1)
    {
      currentMillis = millis();
      myservo.write(90);
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10); 
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distance= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No hay pulso del sensor");
      }
      else 
      {
        Serial.print("Distancia del ultrasónico:");
        Serial.print(distance);
        Serial.println("cm");
        Serial.print(distanceR-distanceL);
         Serial.println("cm");
      }
    }
    if(flag.HCSR04Flag==2) 
    {
      myservo.write(20);
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10);  
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distanceR= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No hay pulso del sensor");
      }
      else 
      {
      }
    }
    if(flag.HCSR04Flag==3)  
    {
      myservo.write(160);
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
       delayMicroseconds(10);  
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distanceL= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No hay pulso del sensor");
      }
      else 
      {

      }
    }
    vTaskDelay(1);
  }
}
