#include <Arduino.h>
#include <SPI.h>
#include <LittleFS.h>
#include <mcp2515.h>
#include <BluetoothSerial.h>
#include <esp_system.h> // esp_reset_reason() - patrz logResetReason()
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h> // kolejka miedzy canTaskFn a logTaskFn - patrz komentarz przy enqueueLine()

// Zwykly ESP32 (potwierdzone przez esptool: "This chip is ESP32, not
// ESP32-S3", NIE AtomS3/ESP32-S3) -> MCP2515, pasywny (listen-only) logger
// CAN 83.333 kb/s do pliku na LittleFS.
//
// UWAGA (2026-09-04) - modul MCP2515 jest zasilany 5V. Do 2026-09-04 linie
// SPI (MISO/MOSI/SCK/CS) byly wpiete WPROST w ESP32 (logika 3,3V) BEZ
// konwertera poziomow - MISO wychodzace z chipu przy 5V przekracza
// dopuszczalne napiecie wejsciowe GPIO ESP32 (~3,3-3,6V max), realny
// kandydat na (wspol)przyczyne przewlekajacej sie korupcji odczytu SPI
// (patrz # STATS invalid_dlc/rx0_ovr w historii nizej i readMessageFast() -
// ta poprawka adresuje inny, potwierdzony software'owy problem, niezalezny
// od tego elektrycznego). Od 2026-09-04 (fizyczna zmiana, nie firmware)
// dodany 8-kanalowy dwukierunkowy konwerter poziomow (VCCA/GND/A0-A7,
// VCCB/GND/B0-B7 - typowy tani modul, prawdopodobnie na bazie BSS138 z
// auto-wykrywaniem kierunku): VCCA -> 3,3V ESP32, VCCB -> te same 5V co
// VCC modulu MCP2515, GND wspolny (ESP32 + MCP2515 + oba GND konwertera +
// masa auta). 4 z 8 kanalow uzyte na CS/SCK/SO/SI (patrz mapa pinow
// nizej), INT (nieuzywany) nie przechodzi przez konwerter w ogole. Piny
// GPIO i protokol SPI ponizej sie NIE zmienily - konwerter jest czysto
// elektryczny, przezroczysty dla firmware'u.
//
// POTWIERDZONE (pierwsze wgranie z konwerterem, 2026-09-04, @ 5MHz): tanie
// auto-wykrywajace konwertery tego typu (rezystory podciagajace + tranzystor
// per kanal) faktycznie nie trzymaja 5 MHz - mcp2515.reset() losowo failuje,
// a # SELFTEST loopback (ramka wewnatrz chipu, zero ruchu na szynie auta)
// tez FAIL. MCP_SPI_CLOCK obnizony do 1 MHz w tym samym wgraniu - patrz
// obszerny komentarz przy MCP_SPI_CLOCK nizej dla pelnej historii,
// WLACZAJAC sprzeczny wynik z 2026-09-02 (inny eksperyment, bez
// konwertera) ktory trzeba bedzie odroznic od tego przy nastepnej jezdzie.
//
// WERSJA 2: sterowanie MCP2515 przez bibioteke autowp/arduino-mcp2515
// (https://github.com/autowp/arduino-mcp2515) zamiast wlasnego kodu na
// golym SPI. Zainstaluj ja w Arduino IDE: Tools -> Manage Libraries ->
// szukaj "autowp-mcp2515" -> Install (albo ZIP z linku wyzej).
//
// WAZNE ZNALEZISKO, ktore tlumaczy wczesniejsza porazke z ta sama
// biblioteka (patrz sesja z 2026-08-29, "MCP2515 bitrate initialization
// failed"): biblioteka NIE MA gotowej tabeli CNF1/CNF2/CNF3 dla
// kombinacji 8 MHz + 83.333 kb/s (mcp2515.h ma taki wpis tylko dla 16 MHz
// i 20 MHz - MCP_16MHz_83k3BPS_CFG*, MCP_20MHz_83k3BPS_CFG*, brak
// MCP_8MHz_83k3BPS_CFG*). Wywolanie mcp2515.setBitrate(CAN_83K3BPS,
// MCP_8MHZ) w tej bibliotece zawsze zwraca ERROR_FAIL, niezaleznie od
// stanu sprzetu - stad tamten blad, nie tylko (albo wcale) problem
// okablowania. Ten sketch omija to, dopisujac rejestry CNF1..CNF3 recznie
// (patrz funkcja writeBitTiming8MHz83k3 nizej), zamiast wolac setBitrate().
//
// UWAGA - GPIO6/7/8 (i 9-11) sa na zwyklym ESP32 NA STALE zarezerwowane dla
// wewnetrznej magistrali SPI do wbudowanego chipu flash. Uzycie ich do
// czegokolwiek innego powoduje natychmiastowy reset przez watchdog
// (TG1WDT_SYS_RESET) zaraz po starcie. Dlatego ponizej sa domyslne piny
// VSPI. Podlacz fizycznie wedlug TEJ mapy.
//
// G5  -> CS
// G18 -> SCK
// G23 -> SI / MOSI
// G19 -> SO / MISO
// G39 -> INT  (nieuzywany - status jest odpytywany w petli, nie przez INT)
//
// WYMAGANE w Arduino IDE: Board = "ESP32 Dev Module" (nie ESP32-S3 Dev
// Module). Tools -> Partition Scheme musi zawierac LittleFS/SPIFFS (np.
// "Default 4MB with spiffs").

// -----------------------------------------------------------------------------
// KTORA MAGISTRALA - jedyne miejsce, ktore zmieniasz przy przepinaniu sondy
// -----------------------------------------------------------------------------
//
// W204 ma dwie osobne magistrale i to SA ROZNE PRZEWODY w innych miejscach
// auta. Jedna plytka z jednym MCP2515 slucha naraz tylko jednej z nich.
//
//   BUS_CANB - magistrala komfortu, 83,333 kb/s. Nadwozie, drzwi, swiatla,
//              klimatyzacja, wyswietlacz licznika. Tu jestesmy dzis.
//
//   BUS_CANC - magistrala napedowa, 500 kb/s. Silnik i skrzynia: temperatura
//              oleju silnika (ramka 0x30D = ECM_A1), temperatura oleju
//              skrzyni (0x2F1 = TCM_A1), cisnienie doladowania. Przewody
//              zielony (CANL) i zielono-bialy (CANH), kostka pod wykladzina
//              przy progu drzwi z przodu.
//
// Jesli szukasz danych o silniku/skrzyni - to jest CAN-C, nie CAN-B, i sonde
// trzeba fizycznie przepiac. Sama zmiana ponizej nie wystarczy.
//
// Przy okazji: przy CAN-C znika caly nasz recznie liczony bit-timing.
// Biblioteka ma gotowy wpis 8 MHz / 500 kb/s (MCP_8MHz_500kBPS_CFG1..3),
// wiec wystarczy zwykle setBitrate() - to wlasnie brak wpisu dla 83,333 kb/s
// zmusil nas kiedys do pisania CNF1..CNF3 recznie.
#define BUS_CANB 0
#define BUS_CANC 1

const uint8_t TARGET_BUS = BUS_CANB;

const uint8_t PIN_CS = 5;

// Rejestry CNF1..CNF3 (adresy z datasheetu MCP2515, biblioteka trzyma je
// prywatnie wiec pisemy je sami surowym SPI, tym samym formatem instrukcji
// WRITE co reszta biblioteki - patrz mcp2515.cpp: startSPI() uzywa
// SPISettings(clock, MSBFIRST, SPI_MODE0), instrukcja WRITE = 0x02).
const uint8_t MCP_INSTR_WRITE = 0x02;
const uint8_t REG_CNF1 = 0x2A;
const uint8_t REG_CNF2 = 0x29;
const uint8_t REG_CNF3 = 0x28;

// BITMOD (0x05) - jak WRITE, ale zmienia tylko bity wskazane w masce,
// zamiast nadpisywac caly rejestr. Potrzebne do CANINTF (0x2C): biblioteka
// nie eksportuje publicznie modifyRegister(), a jej clearInterrupts()
// zawsze zeruje CALY rejestr CANINTF (patrz komentarz przy
// clearOneRxIfBit() nizej) - to kasuje RXnIF takze dla bufora, ktory w tej
// samej chwili moglby miec juz nowa, jeszcze nieprzeczytana ramke.
const uint8_t MCP_INSTR_BITMOD = 0x05;
const uint8_t REG_CANINTF = 0x2C;

// SPI 5 MHz (2026-09-02 wieczorem, COFNIETE z 2 MHz po realnej jezdzie -
// patrz "PLANOWANE" nizej, druga proba tej samej hipotezy nie wychodzi).
//
// Historia w skrocie: 1 MHz -> 5 MHz (naprawilo przepelnienia RXB0/RXB1,
// dalo dosc czasu na opróznienie buforow) -> 2 MHz (proba wg analogii z
// forum Jetson Nano: nizszy zegar SPI = mniej bledow odczytu ID; ta sama
// zmiana co dzis wieczorem zbiegla sie w JEDNYM wgraniu z zamiana
// WiFi->BT, wbrew wlasnej zasadzie z planu "testuj po jednej zmianie na
// raz").
//
// WYNIK REALNEJ JAZDY na 2 MHz (2026-09-02, ~19:13-19:23, 10 min, REC/TEC=0
// przez caly czas - a wiec bez zadnego bledu na fizycznej magistrali):
// invalid_dlc/rx ~23x i rx0_ovr/rx ~4x - GORZEJ niz zarowno baseline sprzed
// tej zmiany na 5MHz bez WiFi (~5x / ~1,8x), jak i baseline z WiFi na
// 5MHz (~12x / ~2,6x), ktory wlasnie probowalismy naprawic. Wniosek:
// hipoteza "nizszy zegar = mniej korupcji" sie tu nie potwierdzila -
// dominujacym czynnikiem na tym okablowaniu jest najwyrazniej czas na
// oproznienie RXB0/RXB1, nie integralnosc sygnalu, dokladnie jak przy
// pierwszej zmianie 1MHz->5MHz. Cofniete do 5 MHz, zeby odizolowac
// zmienna "WiFi->BT" od zmiennej "zegar SPI" (dwie zmiany na raz w
// poprzednim wgraniu to byl blad metodologiczny, patrz wyzej). Jesli po
// tym cofnieciu invalid_dlc/rx0_ovr wroca w okolice starego baseline
// (~5x/~1,8x) na kolejnej jezdzie - to potwierdzi, ze 2MHz bylo realna
// regresja, nie szum pomiarowy. Ta sama predkosc jest przekazywana do
// konstruktora MCP2515 ponizej, zeby biblioteka i nasz reczny zapis CNF
// uzywaly identycznych ustawien magistrali.
// 2026-09-04, obnizone 5 MHz -> 1 MHz PO DODANIU konwertera poziomow
// 3,3V<->5V na liniach SPI (patrz komentarz o okablowaniu na gorze pliku).
// UWAGA na sprzecznosc z historia wyzej: eksperyment z 2026-09-02 (5->2MHz,
// BEZ konwertera) wypadl gorzej (invalid_dlc/rx ~23x, rx0_ovr/rx ~4x) i
// wniosek byl "dominuje czas oproznienia RXB0/RXB1, nie integralnosc
// sygnalu - wolniejszy zegar tylko szkodzi". Ta zmiana nie jest powtorka
// tamtego eksperymentu - to inny objaw, na innym okablowaniu:
//   - Pierwsze wgranie z konwerterem @ 5MHz: mcp2515.reset() losowo failuje
//     (bledy juz na pojedynczych rejestrowych transakcjach init), a kiedy
//     sie uda - # SELFTEST loopback (patrz runLoopbackSelfTest() nizej)
//     rowniez FAIL, z sendErr=OK/rxErr=OK ale zla trescia odczytanej ramki.
//   - Loopback to JEDNA izolowana ramka wewnatrz chipu - zero ruchu na
//     szynie auta, zero rywalizacji o RXB0/RXB1 (bufor nie moze sie
//     przepelnic przy jednej ramce). Jego porazka przy 5MHz przez
//     konwerter wyklucza hipoteze "za wolne oproznianie buforow" jako
//     wyjasnienie TEGO konkretnego niepowodzenia - to wyglada na realna
//     degradacje sygnalu SPI (prawdopodobnie zbyt szybki zegar dla tanich
//     auto-sensingowych konwerterow, patrz komentarz na gorze pliku).
// Test do zrobienia po tej zmianie: (1) # SELFTEST loopback zaraz po
// starcie - jesli teraz wyjdzie OK, to potwierdza teorie integralnosci
// sygnalu; (2) realna jazda - jesli invalid_dlc/rx0_ovr wroca w okolice
// starego baseline sprzed konwertera (~3,5x/~2x wg # STATS z 2026-09-04)
// albo nizej, sukces; jesli zamiast tego eksploduje jak w 2026-09-02 (~23x/
// ~4x), to znaczy ze na TYM okablowaniu (z konwerterem) rowniez dominuje
// czas oproznienia buforow, nie integralnosc - i potrzeba szybszego
// (push-pull, nie auto-sensing) konwertera zamiast dalszego zwalniania.
// 2026-09-05: podniesione 1 MHz -> 2 MHz. Powod: cztery realne jazdy pod
// rzad (dzielnik 10k/20k na MISO + CS/SCK/MOSI bezposrednio, potem to samo
// + krytyczna sekcja w readMessageFast(), potem CS/SCK/MOSI przez kolejne
// kanaly 8-kanalowego konwertera zamiast bezposrednio) dały STATYSTYCZNIE
// IDENTYCZNY wynik: invalid_dlc/rx ~4,2-4,8x, rx0_ovr/rx ~2,7-3,1x, REC/TEC
// caly czas 0. Cztery rozne zmiany elektryczne/software'owe, zero ruchu w
// wyniku - jedyna zmienna, ktorej NIKT nie ruszal od 2026-09-04 (kiedy
// obnizono z 5 MHz z powodu problemow AKURAT tego jednego auto-sensing
// konwertera), to zegar SPI. Przy 1 MHz jedna ramka (14 bajtow: instrukcja+
// naglowek+dane) to ~112us samego przesylu + narzut digitalWrite()/wywolan
// funkcji, rzedu 150-300us - a bufor RXB0/RXB1 (tylko 2 sztuki) moze sie
// zapelnic zanim petla canTaskFn zdazy go opróznic przy realnym ruchu na
// busie. To lepiej tlumaczy plaski wynik u wszystkich czterech testow niz
// dowolna z badanych dotad hipotez elektrycznych/timing-race - one wszystkie
// zakladaly cos zwiazanego z konkretna linia/sekcja kodu, a nie z tym, jak
// szybko w ogole caly odczyt sie wykonuje. Test do zrobienia: realna jazda,
// invalid_dlc/rx i rx0_ovr/rx wzgledem dzisiejszego ~4,4x/~3,0x - jesli
// spadnie, potwierdza teorie predkosci odczytu; jesli mcp2515.reset()/
// SELFTEST zaczna failowac jak przy 5MHz na starym konwerterze, to znaczy ze
// ktoras z linii (najpewniej ta czesc CS/SCK/MOSI, ktora nadal idzie przez
// auto-sensing konwerter) nie trzyma wyzszego zegara - wtedy cofnij do
// 1 MHz i szukaj dalej gdzie indziej, nie w zegarze.
// 2026-09-05, kontynuacja: realna jazda na 2MHz dala invalid_dlc/rx ~4,7-5,1x,
// rx0_ovr/rx ~2,8-2,9x - NIE lepiej niz 1MHz (~4,4-4,6x/~3,0x), jesli juz to
// lekko gorzej. To OSLABIA teorie "za wolny zegar = za wolne opróznianie
// buforow" z komentarza wyzej - gdyby to byla prawda, szybszy zegar powinien
// pomoc, a nie zaszkodzic. Mimo to user chce sprawdzic 5MHz (skok wiekszy niz
// 1->2, moze efekt jest nieliniowy albo 2MHz to za malo, zeby cokolwiek
// zmienic). RYZYKO udokumentowane w historii wyzej: przy 5MHz przez ten sam
// tani auto-sensing konwerter (CS/SCK/MOSI nadal przez niego ida) mcp2515.reset()
// losowo failowal, a # SELFTEST loopback tez failowal - jesli to samo
// powtorzy sie teraz, to potwierdzi ze auto-sensing konwerter (nie zegar per
// se) jest twardym ograniczeniem predkosci, i trzeba cofnac do 2MHz albo
// zamienic ten konwerter na cos push-pull.
// 2026-09-05, konkluzja calej rundy testow zegara: realna jazda na 5MHz dala
// invalid_dlc/rx ~4,4-5,4x, rx0_ovr/rx ~2,7-2,8x - DOKLADNIE ten sam pulap co
// 1MHz i 2MHz. Trzy predkosci, 5-krotny zakres (1->5MHz), reset()/SELFTEST
// przeszly na kazdej z nich (auto-sensing konwerter na CS/SCK/MOSI jednak
// trzyma nawet 5MHz na tym okablowaniu) - zero zmiany w invalid_dlc/rx0_ovr.
// To ZAMYKA teorie "za wolny zegar SPI = za wolne opróznianie buforow" -
// gdyby to byla prawdziwa przyczyna, 5x szybszy odczyt powinien byc widoczny
// w statystykach, a nie jest. Cofniete do 1 MHz (najlepszy/najbezpieczniejszy
// dotychczasowy wynik, zero ryzyka marginalnosci konwertera) - podnoszenie
// zegara dalej nie ma uzasadnienia, szukac przyczyny invalid_dlc trzeba gdzie
// indziej niz w SPI (np. realne obciazenie magistrali/tylko 2 bufory RX
// odpytywane programowo zamiast przez pin INT - patrz rozmowa z uzytkownikiem
// 2026-09-05).
const uint32_t MCP_SPI_CLOCK = 1000000;
SPISettings mcpRawSpi(MCP_SPI_CLOCK, MSBFIRST, SPI_MODE0);

MCP2515 mcp2515(PIN_CS, MCP_SPI_CLOCK);

File logFile;
char logName[24];

// Wlasny licznik rozmiaru biezacego pliku logu i licznik linii do progu
// sprawdzania wolnego miejsca na partycji - patrz writeLog()/openNextLog()
// nizej (2026-09-03, naprawa lfs_write_max_us=54206).
size_t currentFileBytes = 0;
uint32_t linesSinceSpaceCheck = 0;
const uint32_t LFS_SPACE_CHECK_INTERVAL = 200; // co ile linii sprawdzac LittleFS.usedBytes() (skan metadanych, nie tanie)

uint32_t rxCount = 0;
uint32_t invalidDlc = 0;
uint32_t rx0Overflow = 0;
uint32_t rx1Overflow = 0;
uint32_t lastFlush = 0;
uint32_t lastStats = 0;
uint32_t linesSinceFlush = 0;

// Trzy nowe liczniki (2026-09-02, dalszy ciag planu zmian) - patrz
// checkDlcConsistency()/drainMcp2515() i appendToBtBuffer() nizej. Licza
// zdarzenia, ktore JUZ dzis sie dzialy (ramki EXT byly logowane jak
// kazde inne, przepelniony bufor BT cicho gubil linie) - nowe jest tylko
// to, ze teraz jest to widoczne w # STATS zamiast dziac sie po cichu.
uint32_t dlcMismatch = 0;  // ramka DATA z innym DLC niz ostatnio dla tego ID - odrzucona jako uszkodzony odczyt
uint32_t extDropped = 0;   // ramka EXT (29-bit) - na tej magistrali to zawsze artefakt odczytu, nigdy prawdziwe dane
uint32_t rtrDropped = 0;   // ramka RTR - na tej magistrali to (niemal zawsze) tez artefakt odczytu, nie prawdziwe zadanie zdalne - patrz drainMcp2515()
uint32_t btBufferDropped = 0; // linia pominieta bo btTxBuffer byl pelny
uint32_t queueDropped = 0; // linia CAN pominieta, bo logQueue byla pelna (log_task nie nadazal) - patrz nizej

// -----------------------------------------------------------------------------
// Rozdzial na dwa zadania FreeRTOS (2026-09-03) - odczyt SPI z MCP2515 na
// wlasnym rdzeniu, oddzielony od zapisu na LittleFS/BT
// -----------------------------------------------------------------------------
//
// PRZYCZYNA: prawdziwa jazda (nie stacjonarny test) pokazala invalid_dlc
// ~32x wieksze niz rx (jedna sesja: rx=412 invalid_dlc=13234 rx0_ovr=539) -
// znacznie gorzej niz jakikolwiek wczesniejszy baseline z historii tego
// pliku. W surowym logu widac tez powtarzajace sie na przemian RTR/DATA dla
// tego samego ID w odstepach ~50-150us (np. 0x23E) - to wyglada na TEN SAM
// rodzaj artefaktu co juz znany problem z EXT (patrz "MAJOR FINDING" przy
// extDropped), tylko przekłamany bit RTR zamiast bitu IDE. Wspolny
// mianownik: odczyt SPI bufora RXBn "urywa sie" (torn read), bo pomiedzy
// kolejnymi checkReceive() w dawnej wspolnej petli potrafily wykonac sie
// blokujace operacje - zapis na LittleFS (writeLog()/openNextLog() przy
// rotacji) i zapis do stosu BT (btSerial.write() w flushBtBuffer()) -
// dajac czas nowej ramce nadpisac bufor w trakcie jego czytania.
//
// NAPRAWA: caly kontakt z MCP2515 po SPI (drainMcp2515, handleErrors,
// printStats) przeniesiony do WLASNEGO zadania (canTaskFn), przypietego do
// rdzenia 1, ktore NIE ROBI nic poza SPI - zero LittleFS, zero Serial, zero
// BT, wiec nic go nie moze zablokowac na dluzej niz pojedyncza transakcja
// SPI. Gotowe linie (ramki i STATS) trafiaja przez kolejke FreeRTOS
// (logQueue) do DRUGIEGO zadania (logTaskFn) na rdzeniu 0, ktore robi cala
// "wolna" robote (Serial, LittleFS, BT) - to jedyne miejsce, gdzie te
// blokujace wywolania jeszcze wystepuja, ale juz nie w tej samej petli co
// odczyt SPI.
//
// xQueueSend() z timeoutem 0 (nigdy nie czeka) - zadanie CAN ma NIGDY sie
// nie zablokowac na kolejce. Jesli log_task akurat nie nadaza (np. w
// trakcie rotacji pliku), linia jest po prostu pomijana (queueDropped++),
// dokladnie ta sama filozofia co btTxBuffer/appendToBtBuffer wyzej - lepiej
// stracic pojedyncza linie diagnostyczna niz zaryzykowac kolejny torn read.
//
// PRAWDZIWY pin INT (G39, opisany na gorze pliku jako "nieuzywany") CELOWO
// nie jest tu podpiety programowo - nie da sie zdalnie zweryfikowac, czy
// jest fizycznie podlaczony, a zawieszenie zadania na przerwaniu, ktore
// nigdy nie nadejdzie, zablokowaloby caly odczyt CAN calkowicie (gorzej niz
// dzisiejszy stan). Zamiast tego canTaskFn dalej odpytuje checkReceive() w
// petli - ale teraz BEZ konkurencji o czas CPU ze strony LittleFS/BT, wiec
// odstep miedzy kolejnymi sprawdzeniami to pojedyncze mikrosekundy zamiast
// potencjalnie milisekund.
//
// UWAGA - NIEPRZETESTOWANE jeszcze w realnej jezdzie. Po wgraniu porownaj
// invalid_dlc/rx i czestotliwosc naprzemiennych RTR/DATA na tym samym ID z
// dzisiejsza sesja (rx=412 invalid_dlc=13234 rx0_ovr=539) - to jest nowy
// punkt odniesienia do oceny, czy rozdzial zadan cokolwiek dal.
// 480, nie 400 (2026-09-03, po dodaniu backlog_files_sent/backlog_bytes_sent
// do # STATS) - zmierzone (python len()) najgorszy przypadek pelnej linii
// STATS (wszystkie liczniki uint32 na max wartosci) to teraz 450 znakow;
// 400 obcinaloby ja w srodku, gubiac CANINTF/EFLG/REC/TEC na koncu.
const size_t QUEUE_LINE_CAP = 480; // tyle samo co char line[QUEUE_LINE_CAP] w printStats() - najwieksza linia jaka tu powstaje
struct CanLogItem {
  char line[QUEUE_LINE_CAP];
};
QueueHandle_t logQueue = nullptr;

// Wolane tylko z canTaskFn (logResetReason() w setup(), przed startem
// zadan, dalej pisze bezposrednio - patrz tam). Kopiuje line do kolejki i
// NIGDY nie czeka (timeout 0) - patrz duzy komentarz wyzej.
void enqueueLine(const char *line) {
  CanLogItem item;
  snprintf(item.line, sizeof(item.line), "%s", line);
  if (logQueue == nullptr || xQueueSend(logQueue, &item, 0) != pdTRUE) {
    ++queueDropped;
  }
}

// -----------------------------------------------------------------------------
// Bluetooth Classic SPP - podglad na zywo + logi na zadanie (2026-09-02,
// zastapienie WiFi/OTA/MQTT/webhooka z tej samej sesji)
//
// Powod zmiany: udokumentowany, niezalezny od tego projektu konflikt WiFi +
// MCP2515-po-SPI na ESP32 (GitHub espressif/arduino-esp32 #1670 "WiFi +
// MCP2515 = nightmare", #1812) - w praktyce potwierdzony na tym sprzecie w
// jezdzie 2026-09-02 (invalid_dlc/rx ~11-13x z WiFi wlaczonym vs ~5x bez).
// BT Classic ma inny kontroler radiowy niz WiFi, wiec teoretycznie mniej
// koliduje z pollingiem SPI w hot pathie - ale to zalozenie, nie pewnik,
// patrz UWAGA nizej. Przy okazji znika zaleznosc od zasiegu domowego
// WiFi/hotspotu i zwalnia sie flash, ktory zajmowal stos WiFi/OTA/MQTT
// (bylo ~87% flasha zajete).
//
// UWAGA - NIEPRZETESTOWANE jeszcze na sprzecie. Po wgraniu sprawdz jak
// zwykle rx0_ovr/invalid_dlc z # STATS - jesli zaczna rosnac szybciej niz
// przed ta zmiana, najprostszy krok wstecz to zakomentowanie wywolan BT w
// setup()/loop() (btSerial.begin(), handleBtCommands(), flushBtBuffer()) -
// LittleFS logowanie dziala niezaleznie od tego bloku.
//
// Utracone przy tej zmianie: ArduinoOTA (aktualizacja firmware wymaga
// znow kabla USB), zdalny podglad z dowolnego miejsca (MQTT+webhook
// dzialaly tez z trasy przez Tailscale Funnel - integracja z HA opisana w
// atom-can-sniffer status memory NIE dziala juz automatycznie z tej
// plytki; BT dziala tylko w zasiegu, czyli w aucie, ze sparowanym
// telefonem/laptopem w promieniu paru metrow).
// -----------------------------------------------------------------------------

// Nazwa BT zalezna od magistrali - dzieki temu DWIE plytki (jedna na
// CAN-B, druga na CAN-C) widoczne sa osobno przy parowaniu i nic sie nie
// myli, ktora jest ktora (ten sam powod co dawny MQTT_CLIENT_ID).
const char *BT_DEVICE_NAME =
    (TARGET_BUS == BUS_CANC) ? "W204-CAN-C" : "W204-CAN-B";

BluetoothSerial btSerial;

// Pomiar czasu btSerial.write() (2026-09-02, przeglad kodu) - sprawdza
// konkretna, mierzalna hipoteze zamiast domyslu "BT moze kolidowac z SPI":
// zapis do stosu Bluedroid MOZE bloknac, jesli telefon nie odbiera dosc
// szybko, a write() leci w KAZDYM obrocie loop() (flushBtBuffer()) bez
// zadnego pomiaru do tej pory. Jesli po tej zmianie invalid_dlc/rx0_ovr
// koreluje z bt_write_slow rosnacym w tym samym momencie, to bylby
// konkretny mechanizm, nie tylko podejrzenie - a nie zegar SPI (ktory juz
// sprawdzilismy i nie ruszyl invalid_dlc wcale).
uint32_t btWriteMaxUs = 0;      // najdluzszy pojedynczy btSerial.write() w tej sesji
uint32_t btWriteSlowCount = 0;  // ile razy write() trwal > BT_WRITE_SLOW_THRESHOLD_US
const uint32_t BT_WRITE_SLOW_THRESHOLD_US = 3000; // 3ms - przy 83.3kb/s to juz spora czesc czasu miedzy ramkami

// Ten sam pomiar co przy BT (btWriteMaxUs), tylko dla writeLog() (2026-09-03,
// po tym jak wyciecie Serial.print z logTaskFn NIE naprawilo queue_drop -
// wciaz ~70% strat). Podejrzenie: writeLog() woła LittleFS.totalBytes()/
// usedBytes() PRZY KAZDEJ linii (patrz writeLog() nizej), zeby sprawdzic
// prog rotacji - to NIE musi byc tanie na LittleFS (skan metadanych, nie
// zwykly licznik w RAM). Ten pomiar to potwierdzi albo obali, zamiast
// zgadywac drugi raz.
uint32_t lfsWriteMaxUs = 0;
uint32_t lfsWriteSlowCount = 0;
const uint32_t LFS_WRITE_SLOW_THRESHOLD_US = 3000;

// Male, OGRANICZONE ROZMIAREM buforowanie "co sie ostatnio dzialo",
// wysylane przez BT do polaczonego telefonu/laptopa. Ten sam wzorzec co
// dawny bufor do webhooka: STALY rozmiar, po zapelnieniu kolejne linie sa
// PO PROSTU POMIJANE az do nastepnego flushBtBuffer() (zero memmove w hot
// pathie - ta sama lekcja co delay(1)/Serial.print w historii tego
// pliku). Pelny log i tak zawsze jest na LittleFS niezaleznie od tego -
// to tylko wygodny, biezacy podglad, nie archiwum. Flush jest czesty
// (BT_FLUSH_INTERVAL_MS), bo w odroznieniu od dawnego HTTPS/TLS zapis do
// bufora BT nie blokuje na sekundy - nie ma juz powodu czekac na cisze
// magistrali.
const size_t BT_TX_BUFFER_CAP = 4096;
const uint32_t BT_FLUSH_INTERVAL_MS = 200;
// Powyzej tego wypelnienia flushBtBuffer() wysyla NATYCHMIAST, nie czekajac
// na BT_FLUSH_INTERVAL_MS (2026-09-02, punkt 6c z planu zmian) - przy
// nagłym wzroscie ruchu 200ms mogloby wystarczyc, zeby bufor sie zapelnil
// miedzy jednym zaplanowanym flushem a drugim; to daje dodatkowa szanse na
// wyslanie danych zanim appendToBtBuffer() zacznie je gubic.
const size_t BT_FLUSH_EAGER_THRESHOLD = (BT_TX_BUFFER_CAP * 4) / 5; // 80%
char btTxBuffer[BT_TX_BUFFER_CAP];
size_t btTxBufferLen = 0;
uint32_t lastBtFlush = 0;

// Stan wysylki zaleglych plikow logu w tle, kawalek po kawalku - patrz
// driveBacklogSend() nizej. backlogFile pusty (operator bool() == false)
// oznacza "nic teraz nie wysylamy".
File backlogFile;
String backlogPath;
size_t backlogFileSize = 0;
bool backlogHeaderSent = false;
const size_t BT_BACKLOG_CHUNK_SIZE = 2048;
uint32_t backlogFilesSent = 0;
uint32_t backlogBytesSent = 0;

// Bufor jednej linii komendy przychodzacej z telefonu przez BT (patrz
// handleBtCommands() nizej) - komenda moze przyjsc bajt po bajcie miedzy
// kolejnymi obrotami loop(), wiec trzeba ja skladac miedzy wywolaniami.
const size_t BT_CMD_BUFFER_CAP = 32;
char btCmdBuffer[BT_CMD_BUFFER_CAP];
size_t btCmdBufferLen = 0;

// -----------------------------------------------------------------------------
// Bit timing 8 MHz / 83.333 kb/s (biblioteka nie ma na to gotowej tabeli)
// -----------------------------------------------------------------------------

void mcpWriteRegisterRaw(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(mcpRawSpi);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(MCP_INSTR_WRITE);
  SPI.transfer(reg);
  SPI.transfer(value);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

// Wartosci wyliczone z wzoru na TQ z datasheetu MCP2515 (TQ = 2*(BRP+1)/Fosc),
// tak by dac dokladnie 12 us na bit (= 83.333 kb/s):
//   BRP=1 (pole CNF1) -> TQ = 2*2/8MHz = 0.5 us
//   SYNC(1) + PRSEG(7) + PHSEG1(8) + PHSEG2(8) = 24 TQ -> 24*0.5us = 12us
// Podzial PRSEG/PHSEG1/PHSEG2 (7/8/8 TQ) jest przepisany 1:1 z wpisu
// MCP_16MHz_83k3BPS_CFG2/CFG3 z tej samej biblioteki (mcp2515.h) - tam przy
// 16 MHz BRP=3 daje ten sam TQ=0.5us i te same proporcje segmentow, wiec
// sample point (66.7%) jest identyczny jak w tamtym zweryfikowanym wpisie,
// zmienia sie tylko BRP (bo oscylator jest o polowe wolniejszy).
const uint8_t CNF1_8MHZ_83K3 = 0x01; // SJW=00, BRP=000001 (BRP=1)
const uint8_t CNF2_8MHZ_83K3 = 0xBE; // BTLMODE=1, SAM=0, PHSEG1=111, PRSEG=110
const uint8_t CNF3_8MHZ_83K3 = 0x07; // PHSEG2=111

void writeBitTiming8MHz83k3() {
  // Wymaga trybu Configuration (ustawia go mcp2515.reset() domyslnie po
  // resecie sprzetowym MCP2515 - to jest jego stan po wlaczeniu).
  mcpWriteRegisterRaw(REG_CNF1, CNF1_8MHZ_83K3);
  mcpWriteRegisterRaw(REG_CNF2, CNF2_8MHZ_83K3);
  mcpWriteRegisterRaw(REG_CNF3, CNF3_8MHZ_83K3);
}

// Kasuje TYLKO jeden bit CANINTF (rxnifMask = 0x01 dla RXB0, 0x02 dla
// RXB1 - to sa jednoczesnie wartosci MCP2515::CANINTF_RX0IF/RX1IF).
// Odpowiednik biblioteki (modifyRegister) jest prywatny, wiec robimy to
// sami surowym SPI, tym samym formatem co mcpWriteRegisterRaw().
//
// (2026-09-04) Od readMessageFast() nizej ta funkcja nie jest juz wolana z
// drainMcp2515() w normalnym torze - zostaje zdefiniowana na wszelki
// wypadek/do debugowania, ale RXnIF kasuje sie teraz automatycznie jako
// czesc odczytu ramki (patrz komentarz przy readMessageFast).
void clearOneRxIfBit(uint8_t rxnifMask) {
  SPI.beginTransaction(mcpRawSpi);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(MCP_INSTR_BITMOD);
  SPI.transfer(REG_CANINTF);
  SPI.transfer(rxnifMask); // maska - zmien tylko ten bit
  SPI.transfer(0x00);      // nowa wartosc tego bitu - 0 (skasowany)
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

// -----------------------------------------------------------------------------
// (2026-09-04) Odczyt ramki instrukcja "READ RX BUFFER" zamiast
// biblioteki - prawdopodobna prawdziwa przyczyna masowej korupcji z
// prawdziwych jazd (invalid_dlc/rx ~5-6x, identyczne fragmenty bajtow na
// roznych ID)
// -----------------------------------------------------------------------------
//
// ZNALEZISKO: MCP2515::readMessage(rxbn, &frame) w bibliotece
// autowp/arduino-mcp2515 (mcp2515.cpp:647) NIE UZYWA zoptymalizowanej
// instrukcji "READ RX BUFFER" (datasheet MCP2515 DS20001801J, rozdzial
// 12.4), ktora czyta CALA ramke (SIDH..DATA) jednym ciaglym transferem SPI
// i - to jest kluczowe - kasuje RXnIF (CANINTF) automatycznie przy
// podniesieniu CS na KONCU tej samej transakcji (potwierdzone wprost w
// datasheecie, patrz tez github.com/pierremolinaro/acan2515Tiny#3, ktory
// cytuje ten sam akapit). Zamiast tego biblioteka robi AZ CZTERY osobne
// transakcje SPI (kazda z wlasnym CS low->high) na kazda ramke:
//   1. readRegisters(SIDH, 5B)   - generyczny READ (0x03) + adres
//   2. readRegister(CTRL)        - DRUGI READ (0x03) + adres - tylko po to,
//      zeby wyciagnac bit RTR, ktory dla ramek standardowych (jedyne jakie
//      istnieja na tej magistrali) i tak juz jest w SIDL (bit SRR) -
//      odczytany w kroku 1, wiec ten caly krok jest zbedny
//   3. readRegisters(DATA, dlc)  - TRZECI READ (0x03) + adres
//   4. modifyRegister(CANINTF,..) - BITMOD (0x05) - dopiero TU kasuje RXnIF
//
// Skutek: caly odczyt jednej ramki trwa ~4x dluzej niz musi (4x narzut
// instrukcji+adresu zamiast 1x), a RXnIF - flaga, ktora wg datasheetu chroni
// bufor przed nadpisaniem przez nowa ramke - zostaje w praktyce ustawiona
// przez caly ten wydluzony czas zamiast przez jedna krotka, ciagla
// transakcje. Dokladnie ten sam powod, dla ktorego ten sketch juz wczesniej
// ominal biblioteke przy setBitrate() (patrz writeBitTiming8MHz83k3 wyzej,
// tam z innego powodu - brak wpisu CNF dla 8MHz/83.3kb/s - ale ten sam
// wzorzec: biblioteka nie robi tego co trzeba, wiec robimy to sami surowym
// SPI). NIEPRZETESTOWANE jeszcze w realnej jezdzie - nastepny krok to
// porownanie invalid_dlc/rx i rx0_ovr+rx1_ovr z dzisiejszym baseline
// (rx=29239 invalid_dlc=165403 rx0_ovr=71323).
//
// 0x90 = READ RX BUFFER, RXB0, od SIDH. 0x94 = to samo dla RXB1. (0x92/0x96
// pominiete - zaczynalyby od DATA, tu chcemy caly naglowek wlacznie z ID.)
// (2026-09-04, wieczorem) Sekcja krytyczna wokol calej transakcji SPI
// ponizej - podejrzenie po tym, ze invalid_dlc/rx0_ovr zostaje na
// podobnym poziomie (~5-9x) NIEZALEZNIE od okablowania (bezposrednio,
// przez konwerter, przez dzielnik rezystorowy 10k/20k), a REC/TEC z
// samego MCP2515 caly czas 0 - czyli chip twierdzi, ze odebral z szyny
// poprawna ramke (zero bledow na poziomie protokolu CAN), a korupcja
// pojawia sie dopiero PO tym, w drodze przez SPI do ESP32. To wskazuje na
// software/timing, nie elektryke. readMessageFast() robi 13 osobnych
// SPI.transfer() trzymajac CS nisko przez cala sekwencje - bez wylaczenia
// przerwan, tick FreeRTOSa (domyslnie ~1ms) moze wywlaszczyc canTaskFn W
// TRAKCIE tej sekwencji, zostawiajac CS nisko na czas przelaczenia
// (potencjalnie milisekundy) - MCP2515 przez ten czas moze np. zaczac
// nadpisywac bufor kolejna ramka albo transakcja SPI zostaje rozjechana
// miedzy dwa niepowiazane odczyty. portENTER_CRITICAL/portEXIT_CRITICAL
// (spinlock + wylaczenie przerwan na tym rdzeniu) gwarantuje, ze te 13
// transferow wykona sie jako jedna, nieprzerwana calosc. Trzymane
// mozliwie krotko (tylko ta petla, nie SPI.beginTransaction/
// endTransaction) - SPI.transfer() na ESP32 dla pojedynczych bajtow jest
// synchroniczne/pollingowe (nie DMA/przerwaniowe), wiec bezpieczne w
// sekcji krytycznej, nie zablokuje sie czekajac na przerwanie ktore samo
// wlasnie wylaczylismy. NIEPRZETESTOWANE jeszcze - nastepny krok to
// porownanie invalid_dlc/rx z dzisiejszym baseline (~5-9x) na tym samym
// okablowaniu (dzielnik 10k/20k) po tej zmianie.
static portMUX_TYPE spiCriticalMux = portMUX_INITIALIZER_UNLOCKED;

MCP2515::ERROR readMessageFast(bool rxb0, struct can_frame *frame) {
  SPI.beginTransaction(mcpRawSpi);
  portENTER_CRITICAL(&spiCriticalMux);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(rxb0 ? 0x90 : 0x94);
  const uint8_t sidh = SPI.transfer(0x00);
  const uint8_t sidl = SPI.transfer(0x00);
  const uint8_t eid8 = SPI.transfer(0x00);
  const uint8_t eid0 = SPI.transfer(0x00);
  const uint8_t dlcReg = SPI.transfer(0x00);

  // Ten sam limit co biblioteka (CAN_MAX_DLEN=8) - nie czytamy poza to, co
  // faktycznie miesci sie w frame->data[], nawet gdy DLC_MASK (0x0F) daje
  // wiecej (uszkodzony odczyt i tak zostanie odrzucony nizej po can_dlc).
  const uint8_t dlcMasked = dlcReg & 0x0F; // DLC_MASK z biblioteki
  const uint8_t dlcToRead = dlcMasked > 8 ? 8 : dlcMasked;
  for (uint8_t i = 0; i < dlcToRead; ++i) {
    frame->data[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH); // <- RXnIF kasuje sie TU, automatycznie (datasheet 12.4)
  portEXIT_CRITICAL(&spiCriticalMux);
  SPI.endTransaction();

  uint32_t id = (static_cast<uint32_t>(sidh) << 3) + (sidl >> 5);
  const bool extended = (sidl & 0x08) != 0; // TXB_EXIDE_MASK z biblioteki (IDE bit w SIDL)
  if (extended) {
    id = (id << 2) + (sidl & 0x03);
    id = (id << 8) + eid8;
    id = (id << 8) + eid0;
    id |= CAN_EFF_FLAG;
  } else if (sidl & 0x10) {
    // SRR (Standard frame Remote transmit Request) - bit 4 SIDL, wazny
    // tylko gdy IDE=0. Zastepuje readRegister(CTRL) z biblioteki - ten sam
    // fizyczny stan, jeden odczyt mniej.
    id |= CAN_RTR_FLAG;
  }

  frame->can_id = id;
  frame->can_dlc = dlcMasked;

  return dlcMasked > 8 ? MCP2515::ERROR_FAIL : MCP2515::ERROR_OK;
}

// -----------------------------------------------------------------------------
// LittleFS
// -----------------------------------------------------------------------------

// Znajduje plik can_NNNN.log o najmniejszym NNNN (najstarszy wg naszej wlasnej
// numeracji rotacji). Zwraca false, jesli zadnego juz nie ma.
bool findOldestLog(char *outPath, size_t outPathSize) {
  bool found = false;
  unsigned int bestIdx = 0;

  File root = LittleFS.open("/");
  File entry = root.openNextFile();
  while (entry) {
    String name = entry.name();
    unsigned int idx;
    if (sscanf(name.c_str(), "can_%u.log", &idx) == 1) {
      if (!found || idx < bestIdx) {
        bestIdx = idx;
        found = true;
      }
    }
    entry = root.openNextFile();
  }
  root.close();

  if (found) {
    snprintf(outPath, outPathSize, "/can_%04u.log", bestIdx);
  }
  return found;
}

// Usuwa NAJWYZEJ JEDEN najstarszy plik can_NNNN.log, jesli zajete miejsce
// przekracza prog. Bez tego partycja LittleFS zapycha sie starymi logami z
// poprzednich sesji testowych, a kazdy kolejny zapis zaczyna failowac
// ("No more free space") - sterownik LittleFS wypisuje to na Serial przy
// KAZDEJ probie zapisu, co samo w sobie mocno spowalnia petle i pogarsza
// rx0_ovr/invalid_dlc, niezaleznie od predkosci samego pollingu.
//
// POPRAWKA (2026-09-03, po realnej jezdzie): wczesniejsza wersja kasowala
// W JEDNYM WYWOLANIU wszystkie zalegle pliki naraz (petla while). Zmierzone
// na sprzecie: przy zaleglosci 4 plikow pojedyncze wywolanie pruneOldLogs()
// (wolane synchronicznie z writeLog() przy rotacji, w logTaskFn) trwalo
// 1,37 SEKUNDY - queue_drop (dotad zawsze 0) skoczylo od razu o 24, bo
// kolejka (128 slotow) fizycznie nie miala jak wchlonac ramek naplywajacych
// przez ponad sekunde ciszy w odbiorze z kolejki. Koszt rosl liniowo z
// liczba zaleglych plikow - bez gornego ograniczenia byl nieprzewidywalny.
//
// Teraz usuwamy TYLKO JEDEN plik na wywolanie - koszt pojedynczej rotacji
// jest ograniczony do kosztu jednego LittleFS.remove() (rzedu 300-600ms,
// to samo co przy skasowaniu 1-2 plikow w poprzednich, bezpiecznych
// pomiarach), niezaleznie od tego, ile plikow faktycznie trzeba nadrobic.
// Jesli zaleglosc jest wieksza niz jeden plik, kolejne rotacje (co ~3MB
// logu) usuna reszte pojedynczo, po jednym - partycja zostaje przez chwile
// pelniejsza niz docelowe 20% zapasu, ale to bezpieczny kompromis: zapas
// jest i tak spory, a rotacje z duzym ruchem CAN i tak nastepuja czesto.
void pruneOldLogs() {
  const size_t total = LittleFS.totalBytes();
  const size_t freeTargetBytes = total / 5; // zostaw >=20% wolnego miejsca

  uint16_t removed = 0;
  char path[24];

  if (LittleFS.usedBytes() > freeTargetBytes && findOldestLog(path, sizeof(path))) {
    if (LittleFS.remove(path)) {
      ++removed;
    }
  }

  if (removed > 0) {
    Serial.printf("# Usunieto %u starych plikow logu, zeby zwolnic miejsce (used=%u/%u B)\n",
      removed, static_cast<unsigned>(LittleFS.usedBytes()), static_cast<unsigned>(total));
    Serial.flush();
  }
}

bool openNextLog() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }

  // Sprzatanie tylko raz w setup() nie wystarcza - przy duzym ruchu sam
  // BIEZACY, rosnacy plik potrafi zapelnic partycje (ktora jest mniejsza
  // niz 3MB) zanim dojdzie do rotacji po rozmiarze. Zadny plik nie jest
  // teraz otwarty (flush+close powyzej), wiec mozna bezpiecznie skasowac
  // najstarsze.
  pruneOldLogs();

  for (uint16_t i = 0; i < 10000; ++i) {
    snprintf(logName, sizeof(logName), "/can_%04u.log", i);
    if (!LittleFS.exists(logName)) {
      logFile = LittleFS.open(logName, FILE_WRITE);
      break;
    }
  }

  if (!logFile) {
    return false;
  }

  logFile.println("# ESP32 + autowp/arduino-mcp2515 logger");
  logFile.println("# 83.333 kb/s, MCP2515 8 MHz");
  logFile.println("# timestamp_us ID STD|EXT DATA|RTR DLC [DATA]");
  logFile.flush();

  // Wlasny licznik bajtow biezacego pliku (2026-09-03) zamiast pytania
  // logFile.size() przy kazdej linii w writeLog() - zaczyna od dlugosci
  // trzech naglowkow powyzej, zeby currentFileBytes od razu zgadzalo sie z
  // realnym rozmiarem pliku na flashu.
  currentFileBytes = strlen("# ESP32 + autowp/arduino-mcp2515 logger\n")
                    + strlen("# 83.333 kb/s, MCP2515 8 MHz\n")
                    + strlen("# timestamp_us ID STD|EXT DATA|RTR DLC [DATA]\n");
  linesSinceSpaceCheck = 0;

  Serial.printf("# Logging to %s\n", logName);
  Serial.flush();
  return true;
}

void writeLog(const char *line) {
  if (!logFile) {
    return;
  }

  const size_t lineLen = strlen(line);

  // POPRAWKA (2026-09-03, po pomiarze lfs_write_max_us=54206 - patrz
  // lfsWriteMaxUs na gorze pliku): LittleFS.totalBytes()/usedBytes() to
  // skan metadanych calej partycji, NIE tani odczyt licznika w RAM - zmierzone
  // realne zamrożenia writeLog() do >50ms, gdy byly wolane na KAZDA linie
  // (~150-250 razy/s przy realnym ruchu). To byl glowny powod queue_drop
  // (~70% strat mimo wczesniejszych poprawek).
  //
  // Prog rozmiaru pliku (3MB) sprawdzamy teraz przez currentFileBytes -
  // wlasny licznik w RAM, aktualizowany ponizej po kazdym zapisie, zero
  // kosztu SPI/flash. Prog wolnego miejsca na CALEJ partycji (64KB zapasu)
  // sprawdzamy dalej przez LittleFS.totalBytes()/usedBytes(), ale juz nie
  // co linie - tylko co LFS_SPACE_CHECK_INTERVAL linii. To bezpieczne: mamy
  // i tak 20% marginesu wolnego miejsca (patrz pruneOldLogs()), wiec kilkaset
  // linii spoznienia w wykryciu progu (rzedu pojedynczych KB) nie zdazy
  // realnie zapelnic partycji miedzy jednym sprawdzeniem a drugim.
  bool needRotate = currentFileBytes + lineLen > 3UL * 1024UL * 1024UL;
  if (!needRotate) {
    ++linesSinceSpaceCheck;
    if (linesSinceSpaceCheck >= LFS_SPACE_CHECK_INTERVAL) {
      linesSinceSpaceCheck = 0;
      const size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
      if (freeBytes < 64UL * 1024UL) {
        needRotate = true;
      }
    }
  }
  if (needRotate) {
    openNextLog();
    if (!logFile) {
      return;
    }
  }
  logFile.print(line);
  currentFileBytes += lineLen;

  // Jesli zapis sie nie udal (np. partycja pelna), NIE probuj dalej co
  // klatke - sterownik LittleFS przy kazdej nieudanej probie wypisuje
  // wlasny blad na Serial, co przy duzym ruchu CAN samo w sobie potrafi
  // zdominowac czas petli. Zamykamy logFile - kolejne wywolania od razu
  // wracaja przez "if (!logFile)" powyzej, bez kolejnych prob zapisu.
  if (logFile.getWriteError()) {
    Serial.println("# ERROR: zapis do LittleFS nieudany (brak miejsca?) - logowanie do pliku wylaczone do restartu");
    Serial.flush();
    logFile.close();
    return;
  }

  ++linesSinceFlush;
}

void flushLog() {
  uint32_t now = millis();
  if (logFile && (linesSinceFlush >= 25 || now - lastFlush >= 1000)) {
    logFile.flush();
    linesSinceFlush = 0;
    lastFlush = now;
  }
}

// -----------------------------------------------------------------------------
// Bluetooth - strumien na zywo + komenda "wyslij zalegle logi"
// -----------------------------------------------------------------------------

// Dopisuje surowa linie (ten sam tekst co idzie do pliku na LittleFS i na
// Serial) do bufora "ostatnia aktywnosc" wysylanego pozniej przez BT.
// Bufor ma STALY rozmiar (BT_TX_BUFFER_CAP) - kiedy sie zapelni, kolejne
// linie sa PO PROSTU POMIJANE az do nastepnego flushBtBuffer() (zero
// memmove w hot pathie - ta sama lekcja co delay(1)/Serial.print w
// historii tego pliku).
void appendToBtBuffer(const char *line) {
  size_t lineLen = strlen(line);
  if (btTxBufferLen + lineLen >= BT_TX_BUFFER_CAP) {
    // POPRAWKA (2026-09-02, punkt 6c z planu zmian): to gubienie linii
    // dzialo sie tu od zawsze, tylko wczesniej CALKOWICIE po cichu - zaden
    // licznik, zaden slad w # STATS. Teraz przynajmniej widac, ze i ile
    // razy to sie zdarzylo.
    ++btBufferDropped;
    return; // bufor pelny - poczekaj na nastepny flush
  }
  memcpy(btTxBuffer + btTxBufferLen, line, lineLen);
  btTxBufferLen += lineLen;
}

// Wysyla caly nagromadzony bufor za jednym zamachem, jesli jest polaczony
// klient BT. btSerial.write() pisze do bufora stosu Bluedroid (nie na
// powietrze bezposrednio), wiec jest szybkie - stad flush co
// BT_FLUSH_INTERVAL_MS (200ms), a nie raz na minute jak przy dawnym
// webhooku (tam kosztowal blokujacy handshake TLS). Dodatkowo (2026-09-02,
// punkt 6c) flush odpala sie NATYCHMIAST tez wtedy, gdy bufor przekroczy
// BT_FLUSH_EAGER_THRESHOLD - nie trzeba czekac na najblizszy "normalny"
// flush, jesli ruch jest akurat wiekszy niz zwykle.
void flushBtBuffer() {
  uint32_t now = millis();
  bool dueByTime = now - lastBtFlush >= BT_FLUSH_INTERVAL_MS;
  bool dueBySize = btTxBufferLen >= BT_FLUSH_EAGER_THRESHOLD;
  if (!dueByTime && !dueBySize) {
    return;
  }
  lastBtFlush = now;

  if (btTxBufferLen == 0 || !btSerial.hasClient()) {
    return;
  }

  // Pomiar czasu (2026-09-02, przeglad kodu) - patrz komentarz przy
  // btWriteMaxUs na gorze pliku.
  uint32_t writeStart = micros();
  btSerial.write(reinterpret_cast<const uint8_t *>(btTxBuffer), btTxBufferLen);
  uint32_t writeUs = micros() - writeStart;
  if (writeUs > btWriteMaxUs) {
    btWriteMaxUs = writeUs;
  }
  if (writeUs > BT_WRITE_SLOW_THRESHOLD_US) {
    ++btWriteSlowCount;
  }

  btTxBufferLen = 0;
}

// Znajduje najstarszy can_NNNN.log INNY niz aktualnie otwarty plik tej sesji
// (logName) - ten sam kandydat co findOldestLog() (uzywane przez
// pruneOldLogs()), ale z wykluczeniem pliku, do ktorego wciaz trwa zapis.
// Bez tego wykluczenia, na swiezo wystartowanej plytce (jeszcze zaden plik
// nie zdazyl sie zrotowac) jedynym "can_NNNN.log" na LittleFS bylby
// aktualnie pisany plik - probowalibysmy go wtedy wysylac i kasowac w
// trakcie zapisu.
bool startNextBacklogFile() {
  char path[24];
  if (!findOldestLog(path, sizeof(path))) {
    return false;
  }
  if (String(path) == String(logName)) {
    return false; // zostal juz tylko biezacy, aktywnie pisany plik
  }

  File f = LittleFS.open(path, FILE_READ);
  if (!f) {
    return false;
  }
  backlogFile = f;
  backlogPath = String(path);
  backlogFileSize = backlogFile.size();
  backlogHeaderSent = false;
  return true;
}

// -----------------------------------------------------------------------------
// Automatyczna wysylka zaleglych plikow logu w tle, kawalek po kawalku
// (2026-09-03, zastapienie dawnego "wysylaj wszystko na komende FLUSH /
// przy kazdym nowym polaczeniu")
// -----------------------------------------------------------------------------
//
// PRZYCZYNA ZMIANY: dawne sendLogFileOverBt() POMIJALO CALKOWICIE kazdy
// plik >= BT_TX_BUFFER_CAP (4096 B) - a przy realnym ruchu (~120 ramek/s)
// kazdy zrotowany plik (limit 3MB, patrz writeLog()) przekracza 4KB w
// niecala sekunde jazdy. W praktyce oznaczalo to, ze ZADEN kompletny plik z
// realnej jazdy nigdy nie byl wysylany - ani na FLUSH, ani przy
// (re)polaczeniu telefonu - i jedyny sposob na "odzyskanie" czegokolwiek
// bylo utrzymywanie polaczenia (dziala tylko live-bufor BT, patrz
// appendToBtBuffer/flushBtBuffer wyzej) albo pogodzenie sie, ze
// pruneOldLogs() w koncu skasuje plik bez wyslania, gdy zabraknie miejsca -
// to jest to "zapchanie pamieci", ktore wymuszalo reczne rozlaczanie i
// laczenie telefonu, zeby cokolwiek z LittleFS wydostac.
//
// NAPRAWA: kazdy zalegly plik jest teraz strumieniowany w kawalkach po
// BT_BACKLOG_CHUNK_SIZE bajtow, NAJWYZEJ JEDEN kawalek na wywolanie
// driveBacklogSend() (wolane z logTaskFn co obrot petli - patrz tam) -
// dokladnie ta sama filozofia co "usun tylko jeden plik na wywolanie" w
// pruneOldLogs() (ograniczony, przewidywalny koszt pojedynczego kroku,
// niezaleznie od tego ile danych jeszcze zostalo), tylko zastosowana do
// wysylki zamiast kasowania. Dzieki temu logTaskFn dalej normalnie odbiera
// z logQueue miedzy kolejnymi kawalkami i nie traci swiezych ramek z jazdy,
// podczas gdy caly backlog stopniowo znika w tle - bez zadnego recznego
// rozlaczania/laczenia telefonu, nawet przy dlugiej, cieglej jezdzie z
// wieloma rotacjami pliku.
//
// Plik jest kasowany z LittleFS dopiero PO calkowitym wyslaniu - zwalnia to
// miejsce natychmiast po transferze, zamiast czekac az pruneOldLogs()
// usunie go dopiero przy 80% zapelnienia partycji (co wczesniej realnie
// oznaczalo utrate danych, patrz wyzej).
void driveBacklogSend() {
  if (!btSerial.hasClient()) {
    // Klient zniknal w trakcie wysylki tego pliku - nie kontynuuj w tle bez
    // odbiorcy. Plik NIE jest kasowany (wyslano tylko czesc) - zostanie
    // podjety od poczatku przy nastepnej okazji (startNextBacklogFile()
    // zawsze zaczyna caly plik od zera, prosciej niz sledzenie offsetu
    // miedzy rozlaczeniami, a koszt ponownego wyslania jednego pliku jest
    // maly wobec korzysci z prostszego kodu).
    if (backlogFile) {
      backlogFile.close();
    }
    return;
  }

  if (!backlogFile) {
    if (!startNextBacklogFile()) {
      return; // nic zaleglego do wyslania
    }
  }

  if (!backlogHeaderSent) {
    char header[64];
    int headerLen = snprintf(header, sizeof(header), "# --- %s (%u B) ---\n",
      backlogPath.c_str(), static_cast<unsigned>(backlogFileSize));
    if (headerLen > 0) {
      btSerial.write(reinterpret_cast<const uint8_t *>(header), static_cast<size_t>(headerLen));
    }
    backlogHeaderSent = true;
  }

  // static, NIE lokalna tablica na stosie - patrz lekcja z pruneOldLogs()
  // (stack overflow) wczesniej w historii tego pliku.
  static char buf[BT_BACKLOG_CHUNK_SIZE];
  size_t n = backlogFile.readBytes(buf, sizeof(buf));
  if (n > 0) {
    btSerial.write(reinterpret_cast<const uint8_t *>(buf), n);
    backlogBytesSent += static_cast<uint32_t>(n);
  }

  if (n == 0 || !backlogFile.available()) {
    backlogFile.close();
    LittleFS.remove(backlogPath);
    ++backlogFilesSent;
  }
}

// Czyta bajty przychodzace z telefonu przez BT i sklada je w jedna linie
// komendy (moze przychodzic bajt po bajcie miedzy kolejnymi obrotami
// loop(), stad btCmdBuffer jako stan miedzy wywolaniami). "FLUSH" (bez
// wzgledu na wielkosc liter) jest teraz NO-OPEM - zaleglosc wysyla sie sama
// w tle na kazdym polaczeniu, patrz driveBacklogSend() - komenda zostaje
// tylko dla zgodnosci z telefonem (CanSnifferClient.kt dalej ja wysyla przy
// kazdym connect()), zeby nie trzeba bylo jednoczesnie zmieniac obu stron.
// Tanie - kilka porownan bajtow na obrot petli, nawet bez podlaczonego
// klienta available() zwraca 0 od razu.
void handleBtCommands() {
  while (btSerial.available()) {
    char c = static_cast<char>(btSerial.read());
    if (c == '\n' || c == '\r') {
      btCmdBufferLen = 0;
      continue;
    }
    if (btCmdBufferLen + 1 < BT_CMD_BUFFER_CAP) {
      btCmdBuffer[btCmdBufferLen++] = c;
    }
  }
}

// -----------------------------------------------------------------------------
// Odbior ramek (przez biblioteke, nie recznie)
// -----------------------------------------------------------------------------

// Pamiec ostatnio widzianej dlugosci danych (DLC) kazdego STD ID (2026-09-02,
// punkt 6b z planu zmian). Tylko ramki STD DATA trafiaja tutaj - EXT sa
// odrzucane wczesniej w drainMcp2515() (patrz extDropped), a RTR nie niosa
// danych.
//
// STALY rozmiar tablicy (punkt 6e z planu zmian) - ten sam powod co przy
// dawnym frameCache: bez twardego limitu, zle ID (albo dawne EXT-artefakty,
// gdyby kiedys przestaly byc odrzucane wczesniej) moglyby rosnac bez konca
// i wyczerpac RAM - ten sam blad, ktory kiedys juz raz wysadzil stos w
// pruneOldLogs().
//
// POPRAWKA (2026-09-02, przeglad kodu): 64 bylo stanowczo za malo - w
// jednej sesji testowej zaobserwowano 558 ROZNYCH ID na tej magistrali
// (w wiekszosci smieci z uszkodzonych odczytow, patrz dlcMismatch/
// invalid_dlc). Cache zapelnial sie takimi ID w ciagu kilku sekund, po
// czym "fail open" (patrz koniec checkDlcConsistency()) stawal sie norma
// zamiast wyjatku - takze dla PRAWDZIWYCH, powtarzajacych sie sygnalow,
// ktore nie zdazyly zlapac miejsca przed smieciami. 160 pokrywa caly
// znany zestaw ramek z bazy W211 (134) z zapasem, kosztem ~1,3KB RAM -
// nic przy 320KB dostepnych na tej plytce.
const size_t DLC_TRACK_CACHE_SIZE = 160;

struct DlcTrackEntry {
  uint32_t id;
  uint8_t dlc;
  bool used;
};

DlcTrackEntry dlcTrack[DLC_TRACK_CACHE_SIZE];
uint32_t dlcTrackFull = 0; // ile razy zabraklo miejsca w tablicy (diagnostyka, nie krytyczne)

// Zwraca true, jesli ramke nalezy ODRZUCIC. Pierwsze zobaczenie danego ID
// tylko zapamietuje jego DLC (jako najwieksze dotad widziane) i nigdy nie
// odrzuca - nie ma jeszcze z czym porownac. Gdy tablica jest pelna (nowe
// ID, brak wolnego miejsca), rowniez nie odrzuca (fail open, nie fail
// closed) - lepiej przepuscic nieznana ramke, niz ryzykowac odrzucanie
// dobrych danych tylko dlatego, ze akurat zabraklo miejsca w cache'u.
//
// POPRAWKA (2026-09-03): realne ramki na tym ID moga legalnie miec
// KROTSZA dlugosc niz poprzednio widziana (nadajnik czasem przycina
// koncowe bajty zerowe) - to NIE jest uszkodzony odczyt, tylko normalny
// wariant tej samej ramki. Wczesniejsza wersja porownywala do OSTATNIO
// widzianego DLC (nie do stalej/maksimum), wiec kazda naturalna zmiana
// dlugosci w dowolna strone byla odrzucana - a przy naprzemiennym
// dluzsza/krotsza kazda kolejna ramka tego ID byla odrzucana na zmiane
// (to trzymalo np. sensor.olej_w_skrzyni_biegow w HA na sztywno na 0,
// bo aktualizacje nigdy nie docieraly do dekodera). Teraz trzymamy
// NAJWIEKSZE dotad widziane DLC i odrzucamy tylko wtedy, gdy nowa ramka
// jest DLUZSZA niz cokolwiek widziane dotad dla tego ID - to jest realna
// sygnatura uszkodzonego odczytu (przypadkowe bity w polu DLC), nie
// legalna, krotsza wariacja tej samej ramki.
bool checkDlcConsistency(uint32_t id, uint8_t dlc) {
  DlcTrackEntry *slot = nullptr;
  for (size_t i = 0; i < DLC_TRACK_CACHE_SIZE; ++i) {
    if (dlcTrack[i].used && dlcTrack[i].id == id) {
      slot = &dlcTrack[i];
      break;
    }
    if (!dlcTrack[i].used && slot == nullptr) {
      slot = &dlcTrack[i]; // pierwsze wolne, gdyby ID sie nie znalazlo
    }
  }

  if (slot == nullptr) {
    ++dlcTrackFull;
    return false;
  }

  if (!slot->used) {
    slot->used = true;
    slot->id = id;
    slot->dlc = dlc; // najwieksze dotad widziane DLC dla tego ID
    return false;
  }

  if (dlc > slot->dlc) {
    slot->dlc = dlc; // nowe, wieksze maksimum - zapamietaj je na przyszlosc
    return true;      // ale TA ramka jest dluzsza niz wszystko dotychczasowe - odrzuc jako podejrzana
  }

  return false; // taka sama lub krotsza niz dotychczasowe maksimum - akceptuj, to normalny wariant
}

void logFrame(const struct can_frame &frame) {
  const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  const bool rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
  const uint32_t id = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  const uint8_t dlc = frame.can_dlc;

  char payload[25];
  char line[180];
  payload[0] = '\0';

  if (!rtr) {
    size_t pos = 0;
    for (uint8_t i = 0; i < dlc; ++i) {
      int written = snprintf(payload + pos, sizeof(payload) - pos, i == 0 ? "%02X" : " %02X", frame.data[i]);
      if (written < 0 || static_cast<size_t>(written) >= sizeof(payload) - pos) {
        break;
      }
      pos += static_cast<size_t>(written);
    }
  }

  if (rtr) {
    snprintf(line, sizeof(line), "%lu ID=0x%lX %s RTR DLC=%u\n",
      static_cast<unsigned long>(micros()), static_cast<unsigned long>(id),
      extended ? "EXT" : "STD", dlc);
  } else {
    snprintf(line, sizeof(line), "%lu ID=0x%lX %s DATA DLC=%u DATA=%s\n",
      static_cast<unsigned long>(micros()), static_cast<unsigned long>(id),
      extended ? "EXT" : "STD", dlc, payload);
  }

  // ZMIANA (2026-09-03): logFrame() dziala teraz w canTaskFn (patrz duzy
  // komentarz przy enqueueLine() wyzej) - zeby nie ryzykowac zablokowania
  // odczytu SPI na UART/flash/BT, linia tylko trafia do kolejki. Realny
  // Serial.print/zapis na LittleFS/wyslanie przez BT robi logTaskFn na
  // drugim rdzeniu.
  enqueueLine(line);
}

void drainMcp2515() {
  // MCP2515 ma tylko dwa bufory RX (RXB0, RXB1) - max dwie ramki na wejscie.
  //
  // ZMIANA (2026-09-02): jawnie ustalamy TU, ktory bufor (RXB0 czy RXB1)
  // ma dane, zamiast wolac readMessage() bez argumentu. Ta wersja z
  // biblioteki sama w sobie wola getStatus() jeszcze raz, wiec miedzy
  // sprawdzeniem w checkReceive() a odczytem mijaly DWIE osobne transakcje
  // SPI odpytujace status - przy duzym ruchu w tym oknie mogl zdazyc sie
  // zmienic. Odczytujac RXB0/RXB1 explicite (ten sam bajt statusu, ktory
  // wlasnie przeczytalismy) nie ma juz tej niejednoznacznosci.
  //
  // (2026-09-04) rxnifMask (ktory bit CANINTF nalezy skasowac) juz tu nie
  // trzeba liczyc - readMessageFast() kasuje RXnIF sam, jako czesc tej
  // samej transakcji SPI co odczyt ramki, patrz jej komentarz.
  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t stat = mcp2515.getStatus();
    MCP2515::RXBn rxbn;
    if (stat & 0x01) {        // STAT bit0 = RX0IF (MCP2515 READ STATUS format)
      rxbn = MCP2515::RXB0;
    } else if (stat & 0x02) { // STAT bit1 = RX1IF
      rxbn = MCP2515::RXB1;
    } else {
      break;
    }

    struct can_frame frame;
    // (2026-09-04) readMessageFast() zamiast mcp2515.readMessage(rxbn, &frame)
    // z biblioteki - patrz obszerny komentarz przy readMessageFast() wyzej.
    MCP2515::ERROR err = readMessageFast(rxbn == MCP2515::RXB0, &frame);

    if (err == MCP2515::ERROR_OK) {
      // FILTR EXT (2026-09-02, dodatkowy punkt do planu zmian): na tej
      // magistrali (W204 CAN-B, wylacznie 11-bitowe ID) kazda ramka z
      // ustawiona flaga EXT jest juz potwierdzonym artefaktem uszkodzonego
      // odczytu, nie prawdziwa danymi (patrz "MAJOR FINDING" z analizy
      // logow - identyczne fragmenty bajtow pod wieloma roznymi 29-bit ID).
      // Wczesniej takie ramki byly mimo to logowane jak kazde inne - zapis
      // do LittleFS/BT/Serial na kazda z nich to koszt w hot pathie, ktory
      // i tak nigdy nie daje uzytecznych danych. Odrzucamy je TU, przed
      // jakimkolwiek zapisem, zamiast filtrowac dopiero po stronie HA.
      const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
      const bool rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
      const uint32_t id = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

      if (extended) {
        ++extDropped;
      } else if (rtr) {
        // FILTR RTR (2026-09-03, po analizie realnej jazdy): na tej
        // magistrali RTR wystepuje niemal wylacznie jako to samo ID co
        // przed chwila widziana ramka DATA, w odstepie ~50-150us (np.
        // 0x23E, 0x1C2, 0x159 w logu z 2026-09-03) - dokladnie ten sam
        // ksztalt co juz potwierdzony artefakt EXT (przekłamany bit w
        // odczycie, tu akurat bit RTR zamiast IDE), nie prawdziwe zadanie
        // zdalne. Odrzucamy TU, przed jakimkolwiek zapisem/pokazaniem -
        // od tej pory logi/BT/Serial pokazuja wylacznie realne ramki DATA.
        ++rtrDropped;
      } else if (checkDlcConsistency(id, frame.can_dlc)) {
        // ramka dluzsza niz cokolwiek dotad widziane dla tego ID -
        // prawdopodobnie uszkodzony odczyt (patrz checkDlcConsistency()) -
        // odrzucamy z tego samego powodu co EXT powyzej.
        ++dlcMismatch;
      } else {
        ++rxCount;
        logFrame(frame);
      }
    } else {
      // DLC>8 (uszkodzony/niewiarygodny odczyt).
      //
      // (2026-09-04) Od readMessageFast() juz NIE trzeba tu recznie kasowac
      // RXnIF (dawniej clearOneRxIfBit(rxnifMask), bo mcp2515.readMessage()
      // z biblioteki przy ERROR_FAIL zostawial ta flage ustawiona - patrz
      // historia POPRAWKA 2026-09-02 nizej, wciaz opisujaca TAMTEN problem
      // dla kontekstu). readMessageFast() kasuje RXnIF automatycznie przy
      // kazdym odczycie (podniesienie CS), niezaleznie od tego, czy DLC
      // wyszlo poprawne czy nie - bo to ta sama, jedna transakcja SPI.
      //
      // Bez Serial.println() tutaj - przy realnym ruchu to zdarzenie
      // potrafi wystapic dziesiatki razy na sekunde, a kazdy print na UART
      // (115200 bd) blokuje petle na kilka ms, co samo nakreca kolejne
      // przepelnienia RXB0. Licznik i tak trafia do # STATS co 5s.
      ++invalidDlc;
    }
  }
}

void handleErrors() {
  uint8_t eflg = mcp2515.getErrorFlags();

  // Bez Serial.println() na kazde przepelnienie - ten sam powod co przy
  // DROP invalid DLC powyzej (patrz drainMcp2515()).
  if (eflg & MCP2515::EFLG_RX0OVR) {
    ++rx0Overflow;
  }
  if (eflg & MCP2515::EFLG_RX1OVR) {
    ++rx1Overflow;
  }
  if (eflg != 0) {
    // POPRAWKA (2026-09-02): bylo tu mcp2515.clearRXnOVR(), ktore w
    // srodku ROWNIEZ wola clearInterrupts() (caly rejestr CANINTF na 0) -
    // wiec za kazdym razem, gdy EFLG mial ustawiony jakikolwiek bit (a przy
    // przeciazeniu ma prawie caly czas), ta linia potrafila skasowac
    // RXnIF dla ramki, ktora WLASNIE w tej chwili czekala na odczyt w
    // drugim buforze - ten sam problem co w drainMcp2515(), tylko
    // wywolywany raz na kazdy obrot loop(). clearRXnOVRFlags() czysci
    // TYLKO bity RX0OVR/RX1OVR w EFLG, nie rusza CANINTF.
    mcp2515.clearRXnOVRFlags();
  }
}

void printStats() {
  uint32_t now = millis();
  if (now - lastStats < 5000) {
    return;
  }

  // REC/TEC (licznik bledow RX/TX w MCP2515) rosna, gdy chip WIDZI cos na
  // szynie (choc nie musi umiec tego poprawnie zdekodowac), nawet jesli
  // zadna ramka nigdy nie trafia do rxCount. To pozwala odroznic "szyna
  // calkowicie martwa" (REC=0 caly czas) od "cos na szynie jest, ale
  // ramki sie nie dekoduja" (REC>0).
  // Ta sama linia idzie na Serial i przez BT (appendToBtBuffer) - jeden
  // snprintf zamiast dwoch osobnych wywolan, tak jak przy dawnym
  // publishStatsMqtt(), tylko bez oddzielnego formatu JSON dla MQTT.
  char line[QUEUE_LINE_CAP];
  snprintf(line, sizeof(line),
    "# STATS rx=%lu invalid_dlc=%lu rx0_ovr=%lu rx1_ovr=%lu dlc_mismatch=%lu "
    "ext_dropped=%lu rtr_dropped=%lu bt_buf_drop=%lu queue_drop=%lu dlc_track_full=%lu "
    "bt_write_max_us=%lu bt_write_slow=%lu lfs_write_max_us=%lu lfs_write_slow=%lu "
    "backlog_files_sent=%lu backlog_bytes_sent=%lu "
    "free_heap=%lu CANINTF=0x%02X EFLG=0x%02X REC=%u TEC=%u\n",
    static_cast<unsigned long>(rxCount), static_cast<unsigned long>(invalidDlc),
    static_cast<unsigned long>(rx0Overflow), static_cast<unsigned long>(rx1Overflow),
    static_cast<unsigned long>(dlcMismatch), static_cast<unsigned long>(extDropped),
    static_cast<unsigned long>(rtrDropped),
    static_cast<unsigned long>(btBufferDropped), static_cast<unsigned long>(queueDropped),
    static_cast<unsigned long>(dlcTrackFull),
    static_cast<unsigned long>(btWriteMaxUs), static_cast<unsigned long>(btWriteSlowCount),
    static_cast<unsigned long>(lfsWriteMaxUs), static_cast<unsigned long>(lfsWriteSlowCount),
    static_cast<unsigned long>(backlogFilesSent), static_cast<unsigned long>(backlogBytesSent),
    static_cast<unsigned long>(ESP.getFreeHeap()),
    mcp2515.getInterrupts(), mcp2515.getErrorFlags(),
    mcp2515.errorCountRX(), mcp2515.errorCountTX());

  // ZMIANA (2026-09-03): patrz logFrame() - dziala w canTaskFn, wiec tylko
  // enqueueLine(), zaden bezposredni Serial/BT stad.
  enqueueLine(line);

  lastStats = now;
}

// -----------------------------------------------------------------------------
// Autotest Loopback - sprawdza caly potok (bit timing 8MHz/83.3k + SPI +
// biblioteke) BEZ udzialu zewnetrznej szyny CAN, zeby odizolowac problem
// firmware/bit-timingu od problemu "nic nie dociera fizycznie do
// transceivera" (zla szyna, brak zasilania auta, zle piny CANH/CANL).
// -----------------------------------------------------------------------------

bool runLoopbackSelfTest() {
  if (mcp2515.setLoopbackMode() != MCP2515::ERROR_OK) {
    Serial.println("# SELFTEST: nie mozna wejsc w tryb Loopback");
    return false;
  }

  struct can_frame testFrame;
  testFrame.can_id = 0x123;
  testFrame.can_dlc = 2;
  testFrame.data[0] = 0xAA;
  testFrame.data[1] = 0x55;

  MCP2515::ERROR sendErr = mcp2515.sendMessage(&testFrame);
  delay(5);

  struct can_frame rxFrame;
  MCP2515::ERROR rxErr = mcp2515.readMessage(&rxFrame);

  bool ok = sendErr == MCP2515::ERROR_OK &&
            rxErr == MCP2515::ERROR_OK &&
            rxFrame.can_id == testFrame.can_id &&
            rxFrame.can_dlc == testFrame.can_dlc &&
            rxFrame.data[0] == testFrame.data[0] &&
            rxFrame.data[1] == testFrame.data[1];

  Serial.printf(
    "# SELFTEST loopback @ %s: send=%d read=%d -> %s\n",
    TARGET_BUS == BUS_CANC ? "500kb/s/8MHz" : "83.333kb/s/8MHz", sendErr, rxErr, ok ? "OK (chip + bit-timing + SPI dzialaja)" : "FAIL");

  return ok;
}

// -----------------------------------------------------------------------------
// Diagnostyka startu (2026-09-02, punkt 6d z planu zmian)
// -----------------------------------------------------------------------------

// Wypisuje POWOD ostatniego resetu (esp_reset_reason(), ESP-IDF) na Serial
// i - w odroznieniu od zwyklych printf-ow w tym pliku - takze do pliku logu
// i na BT, dokladnie tak jak logFrame()/printStats(). Powod: ta plytka juz
// nie raz resetowala sie sama (stack overflow w pruneOldLogs(), watchdog od
// GPIO6/7/8, przeciazenie petli) i za kazdym razem trzeba bylo miec wtedy
// wlaczony zywy podglad Serial, zeby to zdiagnozowac. Dzieki temu jednemu
// zapisowi PRZYCZYNA widac po fakcie w logu - nie trzeba juz zlapac awarii
// na zywo.
void logResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  const char *nazwa;
  switch (reason) {
    case ESP_RST_POWERON:  nazwa = "POWERON (normalne wlaczenie/podpiecie zasilania)"; break;
    case ESP_RST_EXT:      nazwa = "EXT (reset zewnetrznym pinem/przyciskiem)"; break;
    case ESP_RST_SW:       nazwa = "SW (programowy esp_restart())"; break;
    case ESP_RST_PANIC:    nazwa = "PANIC (crash/wyjatek w firmwarze)"; break;
    case ESP_RST_INT_WDT:  nazwa = "INT_WDT (watchdog przerwan - petla blokowala przerwania zbyt dlugo)"; break;
    case ESP_RST_TASK_WDT: nazwa = "TASK_WDT (watchdog zadania - ten sam objaw co dawne stack-overflow/GPIO6-7-8)"; break;
    case ESP_RST_WDT:      nazwa = "WDT (inny watchdog sprzetowy)"; break;
    case ESP_RST_BROWNOUT: nazwa = "BROWNOUT (spadek napiecia zasilania ponizej progu)"; break;
    case ESP_RST_DEEPSLEEP:nazwa = "DEEPSLEEP (wybudzenie z deep sleep - nieuzywane w tym sketchu)"; break;
    default:                nazwa = "inny/nieznany"; break;
  }

  char line[96];
  snprintf(line, sizeof(line), "# Powod ostatniego resetu: %s (kod %d)\n", nazwa, static_cast<int>(reason));
  Serial.print(line);
  writeLog(line);
  appendToBtBuffer(line);
}

// -----------------------------------------------------------------------------
// Zadania FreeRTOS (patrz duzy komentarz przy enqueueLine() na gorze pliku)
// -----------------------------------------------------------------------------

// Rdzen 1, priorytet 5 - jedyny watek, ktory dotyka SPI/MCP2515 (poza
// setup(), ktory dziala przed startem tego zadania). Nic tu nie blokuje
// dluzej niz pojedyncza transakcja SPI: gdy nie ma danych, vTaskDelay(1)
// oddaje CPU (i karmi watchdog zadania bezczynnego na tym rdzeniu, patrz
// TASK_WDT w logResetReason()) zamiast busy-loopa; gdy dane sa, kolejne
// iteracje leca bez zadnego opoznienia.
void canTaskFn(void *pvParameters) {
  for (;;) {
    if (mcp2515.checkReceive()) {
      drainMcp2515();
    } else {
      vTaskDelay(1);
    }
    handleErrors();
    printStats(); // ma wlasny wewnetrzny rate-limit (co 5s) - tanie wywolanie co iteracje
  }
}

// Rdzen 0, priorytet 2 - cala "wolna" robota (Serial, LittleFS, BT), nigdy
// SPI. xQueueReceive() z timeoutem 50ms sam w sobie pelni role petli -
// budzi sie albo gdy przyjdzie nowa linia, albo najdalej co 50ms, zeby
// flushLog()/flushBtBuffer() (maja wlasne rate-limity czasowe) i
// handleBtCommands() (musi czytac przychodzace bajty na biezaco) dostaly
// szanse dzialac nawet przy calkowitej ciszy na magistrali.
void logTaskFn(void *pvParameters) {
  for (;;) {
    CanLogItem item;
    if (xQueueReceive(logQueue, &item, pdMS_TO_TICKS(50)) == pdTRUE) {
      // POPRAWKA (2026-09-03, realna jazda po rozdziale zadan): Serial.print()
      // NA KAZDEJ ramce bylo usuniete z tego samego powodu, dla ktorego
      // kiedys juz raz bylo krytykowane w logFrame() (patrz stary komentarz
      // "kazdy print to czas na UART") - przy realnym ruchu (~120 ramek/s w
      // tej sesji) UART 115200 bd (~11,5 KB/s) sam niemal wysyca sie samymi
      // printami, a logTaskFn robi to jeszcze OBOK zapisu LittleFS i BT w
      // tej samej petli. Wynik: queue_drop=1347 na rx=1634 (~82%!).
      //
      // POPRAWKA 2 (2026-09-03, zaraz potem): calkowite wyciecie Serial
      // poszlo za daleko - konsola USB (glowny kanal podgladu na biezaco)
      // milkla calkowicie zaraz po "# Ready.", mimo ze dane i tak szly do
      // LittleFS/BT. Kompromis: STATS ('#' na poczatku linii, printStats()
      // ma wlasny rate-limit co 5s) nadal ide na Serial - tanie, rzadkie,
      // przywraca "zywy" podglad liczby rx/invalid_dlc na biezaco. Pojedyncze
      // linie ramek (zaczynaja sie od liczby - timestamp z micros()) NIE ida
      // juz na Serial - to one, nie STATS, wysycaly UART.
      if (item.line[0] == '#') {
        Serial.print(item.line);
      }
      uint32_t lfsStart = micros();
      writeLog(item.line);
      uint32_t lfsUs = micros() - lfsStart;
      if (lfsUs > lfsWriteMaxUs) {
        lfsWriteMaxUs = lfsUs;
      }
      if (lfsUs > LFS_WRITE_SLOW_THRESHOLD_US) {
        ++lfsWriteSlowCount;
      }
      appendToBtBuffer(item.line);
    }
    flushLog();
    flushBtBuffer();
    handleBtCommands();
    driveBacklogSend();
  }
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("# ESP32 + autowp/arduino-mcp2515, 83.3 kb/s polling logger");
  Serial.flush();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI.begin();
  Serial.println("# SPI.begin() OK");
  Serial.flush();

  Serial.println("# LittleFS.begin()...");
  Serial.flush();
  if (!LittleFS.begin(true)) {
    Serial.println("# ERROR: LittleFS mount/format failed - sprawdz Partition Scheme w Tools");
    Serial.flush();
    while (true) { delay(1000); }
  }
  Serial.println("# LittleFS OK");
  Serial.flush();

  // pruneOldLogs() jest teraz wywolywane wewnatrz openNextLog() (przy
  // kazdej rotacji, nie tylko raz tutaj) - patrz komentarz tam.
  if (!openNextLog()) {
    Serial.println("# ERROR: Cannot create log file");
    Serial.flush();
    while (true) { delay(1000); }
  }

  // Plik logu (writeLog()) jest juz otwarty w tym miejscu, wiec powod
  // resetu trafia od razu do trzech miejsc (Serial/plik/BT), nie tylko na
  // Serial - patrz komentarz przy logResetReason() wyzej.
  logResetReason();

  // Bluetooth Classic SPP - patrz duzy komentarz przy BT_DEVICE_NAME na
  // gorze pliku. btSerial.begin() nie blokuje na "polaczenie" jak dawne
  // WiFi.begin() - wystawia radio od razu i wraca, telefon laczy sie
  // kiedy chce (sparowanie z BT_DEVICE_NAME wystarczy raz).
  if (!btSerial.begin(BT_DEVICE_NAME)) {
    Serial.println("# ERROR: Bluetooth SPP nie wystartowal");
  } else {
    Serial.printf("# Bluetooth SPP gotowe, nazwa urzadzenia: %s\n", BT_DEVICE_NAME);
  }
  Serial.flush();

  // reset() sprzetowo resetuje MCP2515 (trafia w tryb Configuration) i
  // przy okazji ustawia RXB0CTRL/RXB1CTRL na odbior WSZYSTKICH poprawnych
  // ramek (STD i EXT, bez filtrow) - dokladnie to, czego potrzebuje
  // pasywny sniffer, wiec nie trzeba tego juz recznie konfigurowac.
  MCP2515::ERROR resetErr = mcp2515.reset();
  Serial.printf("# mcp2515.reset() -> %d (0=OK)\n", resetErr);
  Serial.flush();

  // CAN-C (500 kb/s) ma gotowy wpis w bibliotece, wiec idzie zwyklym
  // setBitrate(). CAN-B (83,333 kb/s) takiego wpisu NIE ma - stad nasz
  // reczny zapis CNF1..CNF3 (patrz komentarz przy CNF1_8MHZ_83K3).
  if (TARGET_BUS == BUS_CANC) {
    MCP2515::ERROR br = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    Serial.printf("# setBitrate(500 kb/s @ 8MHz) -> %d (0=OK)\n", br);
  } else {
    writeBitTiming8MHz83k3();
  }
  Serial.printf("# Magistrala: %s\n",
    TARGET_BUS == BUS_CANC ? "CAN-C (naped, 500 kb/s)"
                           : "CAN-B (komfort, 83,333 kb/s)");
  Serial.flush();

  // Loopback nie wymaga zadnej zewnetrznej szyny - jesli to przejdzie,
  // caly firmware (bit timing + SPI + biblioteka) jest sprawny i "rx=0"
  // w statystykach oznacza, ze sygnal po prostu nie dociera fizycznie do
  // transceivera (zle piny CANH/CANL, wylaczona stacyjka, brak masy itd),
  // a nie blad w tym sketchu.
  runLoopbackSelfTest();
  mcp2515.clearInterrupts(); // sprzata TX0IF zostawione przez selftest

  MCP2515::ERROR modeErr = mcp2515.setListenOnlyMode();
  if (modeErr != MCP2515::ERROR_OK) {
    Serial.printf("# ERROR: Cannot enter listen-only mode (err=%d)\n", modeErr);
    Serial.flush();
    while (true) { delay(1000); }
  }

  Serial.printf("# MCP2515 w trybie listen-only, %s.\n",
    TARGET_BUS == BUS_CANC ? "500 kb/s @ 8 MHz" : "83.333 kb/s @ 8 MHz");
  Serial.println("# Polling status (checkReceive) w dedykowanym zadaniu na rdzeniu 1; INT nieuzywany.");
  Serial.println("# Ready.");
  Serial.flush();

  lastFlush = millis();
  lastStats = millis();

  // Od tego miejsca cala robota dzieje sie w dwoch zadaniach FreeRTOS
  // (patrz duzy komentarz przy enqueueLine() na gorze pliku) - loop()
  // ponizej juz nic nie robi.
  //
  // POPRAWKA (2026-09-03, po realnym teście): probowalismy 256 (2x zapas
  // ponad zmierzone 590ms przy rotacji pliku logu) - xQueueCreate(256, ...)
  // = 256*400B = 100KB ZAWIODLO NA SPRZECIE ("ERROR: xQueueCreate(logQueue)
  // failed"), wolna stera po starcie BT (Bluedroid) na to nie starczyla.
  // Urzadzenie utknelo w petli bledu - ZERO zbierania danych, gorzej niz
  // przed proba zwiekszenia. Cofniete do 128 - to jest SPRAWDZONA, dzialajaca
  // wartosc (queue_drop=0 przez cala dluga jazde w poprzedniej sesji
  // testowej, tylko raz margines byl cienki przy rotacji pliku).
  //
  // Wolny RAM drukujemy TERAZ, przed proba alokacji (nie tylko po), zeby
  // przy ewentualnej kolejnej probie zwiekszenia miec twarda liczbe zamiast
  // zgadywac trzeci raz, ile realnie jest miejsca po starcie BT/LittleFS.
  Serial.printf("# Wolny RAM przed alokacja kolejki (BT/LittleFS juz wystartowane): %lu B\n",
    static_cast<unsigned long>(ESP.getFreeHeap()));
  Serial.flush();

  logQueue = xQueueCreate(128, sizeof(CanLogItem));
  if (logQueue == nullptr) {
    Serial.println("# ERROR: xQueueCreate(logQueue) failed");
    Serial.flush();
    while (true) { delay(1000); }
  }

  xTaskCreatePinnedToCore(canTaskFn, "can_task", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(logTaskFn, "log_task", 8192, nullptr, 2, nullptr, 0);

  // Wolny RAM PO alokacji kolejki i obu zadan - roznica wzgledem liczby
  // powyzej pokazuje realny koszt kolejki (128*400B=~51KB) + stosow zadan
  // (4096+8192B) na TYM konkretnym sprzecie/konfiguracji.
  Serial.printf("# Wolny RAM po starcie zadan: %lu B\n", static_cast<unsigned long>(ESP.getFreeHeap()));
  Serial.flush();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop() {
  // Cala praca przeniesiona do canTaskFn/logTaskFn (utworzonych na koncu
  // setup()) - patrz duzy komentarz przy enqueueLine() na gorze pliku.
  // Domyslne zadanie Arduino (loopTask), ktore wywoluje ta funkcje, nic juz
  // tu nie robi poza spaniem - zostawione zamiast vTaskDelete(NULL), zeby
  // nie polegac na wewnetrznym uchwycie loopTaskHandle Arduino-ESP32.
  vTaskDelay(pdMS_TO_TICKS(60000));
}
