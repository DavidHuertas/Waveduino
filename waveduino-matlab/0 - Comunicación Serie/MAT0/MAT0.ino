  // Tutorial 1: http://www.youtube.com/watch?v=ymWXCPenNM4

void setup() {
  // Inicializamos la comunicación en serie a 9600 bps
  Serial.begin(9600);
  // Comprobamos la comunicación en serie - reconocimiento de rutina
  Serial.println('a'); //Mandando un carácter al PC
  char a = 'b';
  while (a != 'a')
  {
    // Esperar a un carácter determinado del PC
    a=Serial.read();
}
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
