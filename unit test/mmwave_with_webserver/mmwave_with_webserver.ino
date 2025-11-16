// ESP32 × HLK-LD2450 — Enhanced Web Canvas + CSV
// UART: LD2450 TX->GPIO16 (RX2), LD2450 RX->GPIO17 (TX2)
// AP: ESP32-LD2450 / radar2450

#include <WiFi.h>
#include <WebServer.h>

// ---- Config ----
const char* AP_SSID = "ESP32-AP";
const char* AP_PASS = "esp32pass";;
const int WEB_PORT = 80;
const int LED_PIN = 2;
const int UART_RX = 16, UART_TX = 17;
const uint32_t RADAR_BAUD = 256000;
const int SERIAL_BAUD = 115200;
const float X_RANGE_M = 3.0f;   // ±X
const float Y_RANGE_M = 5.0f;   // forward
const unsigned long UI_MS = 100; // page and CSV cadence
const int TRAIL_LEN = 100;      // Configurable trail history length
const int CANVAS_WIDTH = 800;   // Configurable canvas width
const int CANVAS_HEIGHT = 900;  // Configurable canvas height

// ---- Protocol: 30B frame ----
const uint8_t HDR[4] = {0xAA,0xFF,0x03,0x00};
const uint8_t TAIL[2]= {0x55,0xCC};

// ---- State ----
struct Target{bool v; int16_t x,y,vz; uint32_t seen;};
Target T[3];
struct Trail{int16_t X[TRAIL_LEN],Y[TRAIL_LEN];int head=0,n=0;unsigned long last=0;} tr[3];
unsigned long framesSeen=0, framesOK=0, errs=0, webHits=0; 
unsigned long lastTick=0; bool csvHeaderDone=false;

// Simple byte buffer
uint8_t buf[128]; int blen=0;

static inline uint16_t le_u16(const uint8_t* p){ return (uint16_t)p[0] | ((uint16_t)p[1]<<8); }

void pushBytes(){ while (Serial1.available() && blen < (int)sizeof(buf)) buf[blen++] = (uint8_t)Serial1.read(); }

void parseFrames(){
  for(;;){
    if (blen < 30) return;
    int h = -1; for(int i=0;i<=blen-30;i++){ if(buf[i]==HDR[0]&&buf[i+1]==HDR[1]&&buf[i+2]==HDR[2]&&buf[i+3]==HDR[3]){ h=i; break; } }
    if (h < 0){ int drop = blen-29; if(drop>0){ memmove(buf, buf+drop, blen-drop); blen-=drop; } return; }
    if (h+30 > blen){ if(h>0){ memmove(buf, buf+h, blen-h); blen-=h; } return; }
    uint8_t f[30]; memcpy(f, buf+h, 30); int take=h+30; memmove(buf, buf+take, blen-take); blen-=take; framesSeen++;
    if (f[28]!=TAIL[0]||f[29]!=TAIL[1]){ errs++; continue; }
    for(int k=0;k<3;k++){
      int b=4+k*8; uint16_t xr=le_u16(&f[b+0]); uint16_t yr=le_u16(&f[b+2]); uint16_t vr=le_u16(&f[b+4]);
      // Decode per LD2450: X signed via MSB flag, Y = raw-32768 (mm), V signed via MSB flag (cm/s)
      int16_t x = (xr & 0x8000) ? (int16_t)(xr - 32768) : (int16_t)(- (int32_t)xr);
      int16_t y = (int16_t)(yr - 32768);
      int16_t v = (vr & 0x8000) ? (int16_t)(- (int32_t)(vr - 32768)) : (int16_t)vr;
      bool nz = !(xr==0 && yr==0 && vr==0);
      T[k].v=nz; T[k].x=x; T[k].y=y; T[k].vz=v; T[k].seen=millis();
    }
    framesOK++;
  }
}

void sampleTrails(){ unsigned long now=millis(); for(int k=0;k<3;k++){ if(now - tr[k].last >= 120){ tr[k].last=now; tr[k].X[tr[k].head]=T[k].x; tr[k].Y[tr[k].head]=T[k].y; tr[k].head=(tr[k].head+1)%TRAIL_LEN; if(tr[k].n<TRAIL_LEN) tr[k].n++; } } }

// ---- Web ----
WebServer server(WEB_PORT);

String page(){
  webHits++; unsigned long up=millis()/1000UL; unsigned long now=millis();
  String tx="[",ty="[",tv="[",tvld="[",tage="[";
  for(int i=0;i<3;i++){ if(i){tx+=",";ty+=",";tv+=",";tvld+=",";tage+=",";} tx+=String(T[i].x); ty+=String(T[i].y); tv+=String(T[i].vz); tvld+=(T[i].v?"1":"0"); tage+=String(now - T[i].seen); }
  tx+="]"; ty+="]"; tv+="]"; tvld+="]"; tage+="]";
  String trX="[", trY="[", trN="[";
  for(int i=0;i<3;i++){ if(i){trX+=",";trY+=",";trN+=",";} trX+="["; trY+="["; for(int j=0;j<tr[i].n;j++){ if(j){trX+=",";trY+=",";} int idx=(tr[i].head+j)%TRAIL_LEN; trX+=String(tr[i].X[idx]); trY+=String(tr[i].Y[idx]); } trX+="]"; trY+="]"; trN+=String(tr[i].n);} trX+="]"; trY+="]"; trN+="]";

  String h = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<meta http-equiv='refresh' content='1'><title>LD2450</title>";
  h += "<style>body{font-family:monospace;background:#111;color:#0f0;margin:0;padding:18px}h1{margin:0 0 8px;border-bottom:2px solid #0f0}canvas{border:1px solid #0f0;background:#0a0a0a;max-width:100%}.muted{color:#777}.t1{color:#00ff00}.t2{color:#ffff00}.t3{color:#00e5ff}</style>";
  h += "</head><body>";
  h += "<h1>ESP32 × HLK-LD2450</h1>";
  h += "Frames "+String(framesOK)+"/"+String(framesSeen)+" · Err "+String(errs)+" · Hits "+String(webHits)+" · Up "+String(up)+"s<br>";
  h += "AP "+String(AP_SSID)+" @ "+WiFi.softAPIP().toString()+"<br><br>";
  h += "<pre>";
  for(int i=0;i<3;i++){
    h += "<span class='t"+String(i+1)+"'>T"+String(i+1)+": ";
    if(T[i].v){ h += "x="+String(T[i].x)+"mm  y="+String(T[i].y)+"mm  v="+String(T[i].vz)+"cm/s  age="+String(now-T[i].seen)+"ms"; }
    else { h += "—"; }
    h += "</span>\n";
  }
  h += "</pre>";
  h += "<pre class='muted'>History (Y cm):\n";
  for(int t=0;t<3;t++){
    h += "<span class='t"+String(t+1)+"'>T"+String(t+1)+": ";
    int n = tr[t].n; int show = n < 12 ? n : 12;
    for(int i=show-1;i>=0;i--){ int idx=(tr[t].head + TRAIL_LEN - 1 - i) % TRAIL_LEN; h += String(tr[t].Y[idx]/10.0f, 1); if(i) h += ","; }
    h += "</span>\n";
  }
  h += "</pre>";
  h += "<canvas id='cv' width='"+String(CANVAS_WIDTH)+"' height='"+String(CANVAS_HEIGHT)+"'></canvas>";
  h += "<script>const tx="+tx+",ty="+ty+",tv="+tv+",tvld="+tvld+",tage="+tage+";const trX="+trX+",trY="+trY+",trN="+trN+";const Xr="+String(X_RANGE_M)+",Yr="+String(Y_RANGE_M)+";";
  h += "const c=document.getElementById('cv'),g=c.getContext('2d'),W=c.width,H=c.height;g.fillStyle='#0a0a0a';g.fillRect(0,0,W,H);";
  h += "function wx(x){return Math.round((x/1000+Xr)/(2*Xr)*W)};function wy(y){return Math.round(H-(y/1000)/Yr*H)};";
  h += "g.strokeStyle='#1a3a1a';g.lineWidth=1;for(let m=0;m<=Yr;m++){let y=H-(m/Yr)*H;g.beginPath();g.moveTo(0,y);g.lineTo(W,y);g.stroke();}for(let m=-Xr;m<=Xr;m++){let x=((m+Xr)/(2*Xr))*W;g.beginPath();g.moveTo(x,0);g.lineTo(x,H);g.stroke();}";
  h += "g.fillStyle='#0f0';g.beginPath();g.arc(W/2,H,4,0,6.283);g.fill();g.font='12px monospace';g.fillText('Y',6,14);g.fillText('X',W-18,H-6);";
  h += "const col=['#00ff00','#ffff00','#00e5ff'];";
  // Draw trails with gradient
  h += "for(let t=0;t<3;t++){let n=trN[t];if(n>1){for(let i=0;i<n-1;i++){let alpha=(i+1)/n;let x1=wx(trX[t][i]),y1=wy(trY[t][i]),x2=wx(trX[t][i+1]),y2=wy(trY[t][i+1]);g.strokeStyle=col[t]+(Math.round(alpha*255)).toString(16).padStart(2,'0');g.lineWidth=2;g.beginPath();g.moveTo(x1,y1);g.lineTo(x2,y2);g.stroke();}}}";
  // Draw targets with distance labels (fixed velocity arrow direction)
  h += "for(let t=0;t<3;t++){if(!tvld[t])continue;let x=wx(tx[t]),y=wy(ty[t]);g.fillStyle=col[t];g.beginPath();g.arc(x,y,5,0,6.283);g.fill();let xm=tx[t]/1000,ym=ty[t]/1000;let dist=Math.hypot(xm,ym);let v=tv[t]/100.0;let r=Math.hypot(xm,ym),ux=r>1e-3?xm/r:0,uy=r>1e-3?ym/r:0;let ax=x-ux*v*25,ay=y+uy*v*25;g.strokeStyle=col[t];g.lineWidth=2;g.beginPath();g.moveTo(x,y);g.lineTo(ax,ay);g.stroke();let ahx=ax+ux*6+(-uy)*4,ahy=ay-uy*6+(-ux)*4;let bhx=ax+ux*6-(-uy)*4,bhy=ay-uy*6-(-ux)*4;g.beginPath();g.moveTo(ax,ay);g.lineTo(ahx,ahy);g.lineTo(bhx,bhy);g.closePath();g.fill();g.fillText('T'+(t+1)+'('+dist.toFixed(2)+'m)',x+6,y-6);}";
  h += "</script>";
  h += "<p class='muted'>CSV for Arduino Serial Plotter emitted every "+String(UI_MS)+" ms.</p>";
  h += "</body></html>";
  return h;
}

void handleRoot(){ server.send(200,"text/html",page()); }
void handleJSON(){ webHits++; unsigned long now=millis(); String j="{"; j+="\"framesOK\":"+String(framesOK)+",\"framesSeen\":"+String(framesSeen)+",\"errs\":"+String(errs)+",\"targets\":["; for(int i=0;i<3;i++){ if(i) j+=","; j+="{"; j+="\"v\":"+(T[i].v?String("true"):String("false")); j+=",\"x\":"+String(T[i].x)+",\"y\":"+String(T[i].y)+",\"vz\":"+String(T[i].vz)+",\"age\":"+String(now-T[i].seen)+"}";} j+= "]}"; server.send(200,"application/json",j);} 

void startAP(){ WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID,AP_PASS); }

void setup(){
  pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  startAP(); server.on("/",handleRoot); server.on("/json",handleJSON); server.begin();
}

void loop(){
  pushBytes(); parseFrames(); sampleTrails(); server.handleClient();
  unsigned long now = millis();
  if (now - lastTick >= UI_MS){
    lastTick = now;
    if (!csvHeaderDone){ Serial.println("t1x,t1y,t1v,t1age,t2x,t2y,t2v,t2age,t3x,t3y,t3v,t3age"); csvHeaderDone=true; }
    unsigned long a0 = now - T[0].seen, a1 = now - T[1].seen, a2 = now - T[2].seen;
    Serial.print(T[0].x); Serial.print(","); Serial.print(T[0].y); Serial.print(","); Serial.print(T[0].vz); Serial.print(","); Serial.print(a0); Serial.print(",");
    Serial.print(T[1].x); Serial.print(","); Serial.print(T[1].y); Serial.print(","); Serial.print(T[1].vz); Serial.print(","); Serial.print(a1); Serial.print(",");
    Serial.print(T[2].x); Serial.print(","); Serial.print(T[2].y); Serial.print(","); Serial.print(T[2].vz); Serial.print(","); Serial.println(a2);
  }
}

