#include <Arduino.h>
// Воспроизведение: горизонтальная СТРОКА через мультиплекс. 3 фазы: BRIGHT(шум,локация)/
// ROW(мультиплекс строки)/BLANK. Вытяжка (ROW-BLANK) вычитанием даёт чистую строку.
#define PIN_SIN 23
#define PIN_CLK 18
#define PIN_LAT 22
#define PIN_BLK 21
#define MIRROR_X true
#define VFD_GRIDS 43
#define VFD_BYTES 30
static uint8_t framebuffer[32][16];
static uint8_t frames[VFD_GRIDS][VFD_BYTES];
uint32_t lfsr=0xACE1u;
static inline int rb(){ lfsr^=lfsr<<13; lfsr^=lfsr>>17; lfsr^=lfsr<<5; return lfsr&1; }
void build_frame(uint8_t* buf,int grid){
  memset(buf,0,VFD_BYTES);
  int cs=grid*3,off[3];
  if(grid%2==0){off[0]=0;off[1]=2;off[2]=4;} else {off[0]=5;off[1]=3;off[2]=1;}
  for(int row=0;row<32;row++) for(int i=0;i<3;i++){
    int fbc=cs+i; if(fbc>=128)continue; int rx=MIRROR_X?(127-fbc):fbc;
    if(framebuffer[row][rx/8]&(0x80>>(rx%8))){ int bp=row*6+off[i]; buf[bp/8]|=(0x80>>(bp%8)); }
  }
  int gb=192+grid; buf[gb/8]|=(0x80>>(gb%8));
  int ng=gb+1;     buf[ng/8]|=(0x80>>(ng%8));
}
static inline void shift30(const uint8_t*f){
  for(int b=0;b<VFD_BYTES;b++){ uint8_t v=f[b];
    for(int k=0;k<8;k++){ digitalWrite(PIN_SIN,(v&0x80)?HIGH:LOW); v<<=1;
      digitalWrite(PIN_CLK,HIGH); digitalWrite(PIN_CLK,LOW); }
  }
}
void px(int x,int y){ if(x<0||x>=128||y<0||y>=32)return; framebuffer[y][x/8]|=(0x80>>(x%8)); }
int rowY=13;
void rebuildRow(){
  memset(framebuffer,0,sizeof(framebuffer));
  for(int y=rowY;y<rowY+4 && y<32;y++) for(int x=0;x<128;x++) px(x,y);   // горизонт. строка (4px)
  for(int g=0;g<VFD_GRIDS;g++) build_frame(frames[g],g);
}
void scanRow(){
  for(int g=0;g<VFD_GRIDS;g++){
    digitalWrite(PIN_BLK,HIGH);
    shift30(frames[g]);
    digitalWrite(PIN_LAT,HIGH); delayMicroseconds(2); digitalWrite(PIN_LAT,LOW);
    digitalWrite(PIN_BLK,LOW);
    delayMicroseconds(40);
  }
}
void setup(){
  pinMode(PIN_SIN,OUTPUT); pinMode(PIN_CLK,OUTPUT);
  pinMode(PIN_LAT,OUTPUT); pinMode(PIN_BLK,OUTPUT);
  rebuildRow();
}
void loop(){
  uint32_t ph=(millis()/600)%3;
  if(ph==0){ for(int i=0;i<3000;i++){ digitalWrite(PIN_SIN,rb()); digitalWrite(PIN_CLK,rb());
      digitalWrite(PIN_LAT,rb()); digitalWrite(PIN_BLK,rb()); } }   // BRIGHT
  else if(ph==1){ scanRow(); }                                     // ROW
  else { digitalWrite(PIN_BLK,HIGH); digitalWrite(PIN_LAT,LOW); delay(2); }  // BLANK
}
