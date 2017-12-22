#define B_CLOCK  12  // horloge
#define B_RESET  13  // réinitialisation
#define B_RD     2   // lecture
#define B_WR     3   // écriture

#define BITS_ADDR  6  // nombre de bits d'adresse
#define CLKDELAY   40  // delai horloge

// Bus d'adresse
//                   bit       0  1  2  3  4  5
int pinsAddress[BITS_ADDR] = {A0,A1,A2,A3,A4,A5};

// Bus de données
//               bit   0 1 2 3 4 5  6  7
int pinsData[8]     = {4,5,6,7,8,9,10,11};

// Variable pour la routine d'interruption
volatile int doread=0;

// Variable pour la routine d'interruption
volatile int dowrite=0;

// Routine d'interruption
void readISR() {
  doread=1;
}

// Routine d'interruption
void writeISR() {
  dowrite=1;
}


// Fonction horloge
void doClock(unsigned int n) {
  for(int i=0; i<n; i++) {
    digitalWrite(B_CLOCK, HIGH);
    delay(CLKDELAY);
    digitalWrite(B_CLOCK, LOW);
    delay(CLKDELAY);
  }
}

// Réinitialisation
void doReset() {
    digitalWrite(B_RESET, LOW);
    doClock(5);
    digitalWrite(B_RESET, HIGH);
}

// Lecture de l'adresse
unsigned int getaddr() {
  unsigned int addr = 0;

  for(int pin=0; pin<BITS_ADDR; pin++) {
      if(digitalRead( pinsAddress[pin] ) == HIGH )
        addr |= (1 << pin);
  }
  
  return addr;
}

// Définition des bits de données
void setData(unsigned char data) {
  for( int pin = 0; pin < 8; pin++ )
    pinMode(pinsData[pin], OUTPUT);

  for(int pin=0; pin<8; pin++) {
    if(data & 1)
      digitalWrite(pinsData[pin], HIGH);
    else
      digitalWrite(pinsData[pin], LOW);
    data >>= 1;
  }
}

// Lecture des  bits de données
unsigned char getData() {
  //delay(10);
  unsigned char data = 0;
  for(int pin=0; pin<8; pin++) {
    if(digitalRead( pinsData[pin] ) == HIGH ) {
      data |= (1 << pin);
    }
  }
  return data;
}

// Configuration
void setup() {
  pinMode(B_CLOCK, OUTPUT);
  pinMode(B_RESET, OUTPUT);
  pinMode(B_RD, INPUT);
  pinMode(B_WR, INPUT);

  // Tout le bus de données en entrée par défaut
  // On le met en sortie juste avant écriture
  for( int pin = 0; pin < 8; pin++ )
    pinMode(pinsData[pin], INPUT);

  // Tout le bus d'adresse en entrée
  for( int pin = 0; pin<BITS_ADDR; pin++ )
    pinMode(pinsAddress[pin], INPUT);

  // Activation moniteur série
  Serial.begin(115200);
  delay(25);

  // Reset du Z80
  Serial.println("Reset Z80");
  doReset();
  delay(10);

  // On attache les interruptions
  // Un changement d'état haut vers bas appelle nos routines
  attachInterrupt(digitalPinToInterrupt(B_RD), readISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(B_WR), writeISR, FALLING);
}

// Mémoire pour notre Z80
unsigned char mmem[] = {
  0xc3, 0x04, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x00, 0x00
};

unsigned char mem[] = {
  0x31, 0x40, 0x00, 0x3a, 0x1a, 0x00, 0x3c, 0x32, 0x1a, 0x00, 0x00, 0x00, 0xf5, 0x00, 0xf1, 0x00,
  0xcd, 0x16, 0x00, 0xc3, 0x03, 0x00, 0x00, 0x00, 0x00, 0xc9, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// Boucle principale
void loop() {
  unsigned int addr;

  if(doread) {
    // Récupération de l'adresse
    addr = getaddr();
    // Affichage
    Serial.print("Adresse: 0x");
    if(addr < 0x10)   Serial.print("0");
    if(addr < 0x100)  Serial.print("0");
    if(addr < 0x1000) Serial.print("0");
    Serial.print(addr, HEX);

    // Si l'adresse est valide (dans les 64 octets de mémoire)
    if(addr >= 0 && addr < sizeof(mem)) {
      // On "pousse" les données correspondante sur le bus
      setData(mem[addr]);
      // Et on affiche le tout
      Serial.print(" [0x");
      if(mem[addr] < 0x10)   Serial.print("0");
      Serial.print(mem[addr], HEX);
      Serial.println("]");
    } else {
      // Nous sommes hors de l'espace mémoire émulé
      // On renvoi 0x00, l'instruction NOP
      setData(0x00);
      Serial.println(" !!!!!");
    }
    // On confirme la prise en charge
    doread=0;
  } else if(dowrite) {
    // Récupération de l'adresse
    addr = getaddr();
    // Affichage
    Serial.print("Adresse: 0x");
    if(addr < 0x10)   Serial.print("0");
    if(addr < 0x100)  Serial.print("0");
    if(addr < 0x1000) Serial.print("0");
    Serial.print(addr, HEX);

    // Si l'adresse est valide (dans les 64 octets de mémoire)
    if(addr >= 0 && addr < sizeof(mem)) {
      // On "place" les données correspondante du bus en mémoire
      mem[addr] = getData();
      // Et on affiche le tout
      Serial.print(" [0x");
      if(mem[addr] < 0x10)   Serial.print("0");
      Serial.print(mem[addr], HEX);
      Serial.println("] W");
    } else {
      // Nous sommes hors de l'espace mémoire émulé
      Serial.println(" !!!!! W");
    }
    // On confirme la prise en charge
    dowrite=0;
  }

  // Impulsion horloge
  doClock(1);
  // bus de données en entrée
  for( int pin = 0; pin < 8; pin++ )
    pinMode(pinsData[pin], INPUT);  // input
}
