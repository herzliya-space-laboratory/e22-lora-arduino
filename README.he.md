<div dir="rtl">

# E22 LoRa UART

[English](README.md)

ספריית ארדואינו למודולי ה-LoRa של Ebyte מסדרת E22 עם ממשק UART. אנחנו משתמשים בה בכרטיס
התקשורת של הלוויין, שיש בו ATmega328PB ומודול E22-400T30D. על השולחן היא רצה על Arduino Uno, והיא
מתקמפלת גם ל-ESP32.

זה רק הדרייבר של המודול. הלוגיקה של הביקון והמשיב שרצה על הלוויין היא sketch נפרד שמשתמש
בספרייה הזאת.

## על המודול

ה-E22 הוא לא שבב LoRa עם ממשק SPI. זה לוח קטן עם מיקרו-בקר משלו שיושב לפני שבב רדיו SX1268.
מדברים עם המיקרו-בקר הזה דרך פורט טורי ב-9600 באוד. חוץ מזה יש שלושה פינים בשימוש: M0 ו-M1
בוחרים את מצב העבודה, ו-AUX מראה אם המודול עסוק (LOW) או פנוי (HIGH). אין NSS, אין DIO0,
ואין גישה ישירה לרגיסטרים של הרדיו.

למודול ה-DIP יש שבעה פינים:

| פין | שם | הערות |
|-----|------|------------------------------------------------------------|
| 1 | M0 | ביט 0 של המצב. אסור להשאיר צף. |
| 2 | M1 | ביט 1 של המצב. אסור להשאיר צף. |
| 3 | RXD | כניסה טורית, לוגיקה של 3.3V |
| 4 | TXD | יציאה טורית |
| 5 | AUX | יציאת עסוק/פנוי. לא לנהוג אותו. |
| 6 | VCC | 5V להספק מלא. בשידור ה-T30D מושך עד 620mA. |
| 7 | GND | |

מצבי עבודה, נבחרים עם M1 ו-M0:

- `0 0` שידור. שליחה וקבלה רגילות.
- `0 1` wake-on-radio. לא בשימוש אצלנו.
- `1 0` תצורה. במצב הזה הפורט הטורי הוא תמיד 9600 8N1.
- `1 1` שינה.

התצורה היא תשעה בייטים של רגיסטרים: כתובת, מזהה רשת, קצב באוד וקצב אוויר, גודל פקטה והספק,
ערוץ, ביטים של אפשרויות, ומפתח הצפנה. קוראים אותם עם `C1 addr len`. כותבים אותם עם
`C0 addr len data` (נשמר בפלאש) או `C2 addr len data` (זמני). בסעיף 6 של המדריך שב-`docs/` יש את
מפת הרגיסטרים המלאה.

כמה עובדות על המודול שמשפיעות על התכנון:

- אי אפשר לקבוע את ה-Spreading Factor ישירות. למודול יש "קצב אוויר" בין 2.4 ל-62.5 kbps, וכל
  קצב הוא צירוף SF/BW קבוע ש-Ebyte בחרו. `setAirRate()` היא השליטה היחידה שיש לנו.
- הצד השני של הקישור כנראה צריך להיות גם E22. Ebyte מוסיפים מעל LoRa פרוטוקול ו-FEC משלהם,
  ולא מצאנו מישהו שפענח את זה עם SX127x רגיל. דוד אומר שקלט את זה פעם עם חומרה אחרת. נבדוק
  את זה במקום להניח.
- התדר הוא מספר ערוץ: 410.125 MHz ועוד ערוץ × 1 MHz, ערוצים 0 עד 83. המשדר-מקלט הראשי של
  הלוויין נמצא על 436.4 MHz, אז אסור להשתמש בערוצים 25 עד 28. הערוץ הוא הגדרה בזמן ריצה, לא
  קבוע.
- המודול לא שומר על גבולות של פקטות בצד הטורי. שתי פקטות שנשלחו אחת אחרי השנייה יכולות לצאת
  מה-UART כזרם אחד. פורמט הפקטה שלנו צריך בייטים של סנכרון, שדה אורך ו-checksum משלו.

## שימוש

<div dir="ltr">

```cpp
#include <E22.h>
#include <SoftwareSerial.h>

// Flight board: module on D2/D3, M0 on D8, M1 on D26 (PE3), AUX not connected.
SoftwareSerial radioSerial(2, 3);      // RX, TX
E22 radio(radioSerial, 8, 26, -1);     // port, M0, M1, AUX. Optional 5th argument: RESET pin.

void setup() {
  Serial.begin(115200);
  radio.begin(9600);

  E22Config cfg;
  radio.readConfig(cfg);
  cfg.channel = 23;            // 433.125 MHz
  cfg.rssiByte = true;         // the module adds an RSSI byte to each received packet
  radio.writeConfig(cfg);      // writes, reads back, compares

  radio.send("hello\n");
}

void loop() {
  int b = radio.readByte();
  if (b >= 0) Serial.write(b);
}
```

</div>

כל קריאת תצורה מעבירה את המודול למצב תצורה, מבצעת את הפעולה, ומחזירה אותו. לכל המתנה על AUX
יש timeout, והקריאה מחזירה `false` אם הוא פג. שום דבר בספרייה לא נתקע לנצח. זה חשוב כי הדבר
היחיד שה-OBC יכול לעשות לכרטיס הזה הוא לכבות ולהדליק אותו.

הספרייה רק מעבירה בייטים. מסגור (framing), checksum ומונים שייכים ל-sketch שמשתמש בה.

### דוגמאות

להריץ בסדר הזה על לוח חדש:

1. `ReadConfig` מדפיס את כל הרגיסטרים ואת גרסת הקושחה. אם זה עובד, החיווט נכון.
2. `PingPong` צריך שני לוחות. אחד שולח PING, השני עונה PONG. שניהם מדפיסים RSSI.
3. `AirRateSweep` הוא ניסוי "שינוי SF כל 10 שניות" מהמסמך של הפרויקט, עם קצב אוויר במקום.
   שני הלוחות עוברים על ששת הקצבים בסנכרון, מרגע ההדלקה.

בכל דוגמה יש בהתחלה בלוק שבוחר את הפורט הטורי ואת הפינים לכל יעד. הדרייבר מקבל HardwareSerial
או SoftwareSerial.

| יעד | פורט טורי לרדיו | M0 | M1 | AUX |
|-----|-----------------|----|----|-----|
| לוח הטיסה ATmega328PB (J3, לפי הסכמה) | SoftwareSerial, ‏D2 ← TXD, ‏D3 → RXD | D8 (PB0) | D26 (PE3) | לא מחובר |
| Arduino Uno (שולחן) | SoftwareSerial, ‏D10 ← TXD, ‏D11 → RXD | D4 | D5 | D6 |
| ESP32 (שולחן) | Serial2, ‏GPIO16 ← TXD, ‏GPIO17 → RXD | GPIO32 | GPIO33 | GPIO34 |

פלט הדיבאג יוצא ל-`Serial` ב-115200 בכל היעדים. ב-Uno להשאיר את D0 ו-D1 פנויים: כל דבר שמחובר
אליהם חוסם צריבה.

ל-PingPong ול-AirRateSweep צריך לוח אחד שנבנה כשולח ואחד כמקבל. לשנות את `ROLE_SENDER`
בהתחלה, או לקבוע אותו משורת הפקודה:

<div dir="ltr">

```
arduino-cli compile -u -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno --library . \
  --build-property "compiler.cpp.extra_flags=-DROLE_SENDER=0" examples/PingPong
```

</div>

### API

| קריאה | תיאור |
|------|-------------|
| `begin(baud, rx, tx)` | פותח את הפורט, נכנס למצב שידור, מחכה ל-AUX. rx/tx בשימוש רק ב-ESP32. |
| `setMode(E22Mode)` | קובע M0/M1, מחכה ל-AUX, ומעביר את הפורט הטורי ל-9600 במצב תצורה |
| `waitIdle(timeoutMs)` | מחכה עד ש-AUX גבוה |
| `hardReset()` | נותן פולס ל-RESET, אם ניתן פין RESET. למודול ה-DIP אין פין RESET. |
| `readConfig(cfg)` / `writeConfig(cfg, persist)` | כל תשעת הרגיסטרים. `persist=false` משתמש ב-`C2`, אז השינוי נמחק אחרי כיבוי. |
| `setChannel`, `setAirRate`, `setTxPower`, `setAddress`, `setRssiByte`, `setAmbientRssi` | שינוי של שדה אחד |
| `atCommand(cmd, reply, len)`, `readFirmwareVersion(buf, len)` | פקודות AT, במצב תצורה. `false` אם המודול דוחה את הפקודה (`FF FF FF` או `ERR`). |
| `send(buf, len)`, `send("text")` | שולח בחלקים בגודל פקטה, מחכה ל-AUX בין החלקים |
| `sendTo(addr, ch, buf, len)` | מוסיף את הכותרת של מצב fixed-point. דורש `cfg.fixedPoint`. |
| `available()`, `read()`, `readByte()`, `flushInput()` | בייטים גולמיים מהמודול |
| `readAmbientRssi(dbm)`, `readLastPacketRssi(dbm)` | דורשים `cfg.ambientRssi`, רק במצב שידור |
| `E22::rssiByteToDbm(b)` | ממיר את בייט ה-RSSI ל-dBm. בייט 0 הוא לא מדידה תקפה ומחזיר 0. |

רמות הספק: `E22TxPower::Level0` היא המקסימום בכל מודול. `dBm30`..`dBm21` ו-`dBm22`..`dBm10` הם
שמות לאותם ארבעה קודים במודולים של 30 dBm ו-22 dBm.

## בנייה

ללוח הטיסה צריך את MiniCore, שמוסיף את ה-ATmega328PB:

<div dir="ltr">

```
arduino-cli config add board_manager.additional_urls https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
arduino-cli core install MiniCore:avr
arduino-cli compile --fqbn MiniCore:avr:328:variant=modelPB,clock=16MHz_external --library . examples/ReadConfig
```

</div>

ל-Arduino Uno על השולחן:

<div dir="ltr">

```
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno --library . examples/ReadConfig
```

</div>

ל-ESP32 על השולחן:

<div dir="ltr">

```
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 --library . examples/ReadConfig
```

</div>

ה-CI מקמפל כל דוגמה ל-ATmega328PB, ל-Uno ול-ESP32 בכל push.

## הערות ומגבלות

- ל-ATmega יש 2KB של RAM. להשתמש בבאפרים קטנים, לשים מחרוזות ב-`F()`, לא להשתמש ב-`printf`.
- ה-ATmega עובד ב-5V וה-RXD של המודול הוא לוגיקה של 3.3V. בלוח הטיסה RXD עובר דרך מחלק מתח
  של 1.3k / 2.7k (בערך 3.4V). M0 ו-M1 מקבלים 5V ישירות, וזה עבד על השולחן. כשמחווטים Uno, לשים
  את אותו מחלק על RXD.
- בלוח הטיסה AUX לא מחובר למיקרו-בקר, אז הדרייבר מחכה זמנים קבועים במקום לחכות ל-AUX (בערך
  12ms לכל המתנה, בערך 26ms להחלפת מצב). זה עוד לא נבדק על חומרה; בכל הבדיקות על השולחן AUX
  היה מחובר.
- כש-`rssiByte` מופעל, המודול מוסיף את בייט ה-RSSI אחרי כל פקטת רדיו. הודעה ארוכה מגודל הפקטה
  (240 בייט כברירת מחדל) מגיעה עם בייט RSSI אחרי כל 240 בייט, לא רק בסוף. קוד המסגור צריך
  להתחשב בזה.
- תצורה זמנית (`C2`) נשמרת ב-RAM של המודול. איפוס של המיקרו-בקר לא מוחק אותה; רק כיבוי והדלקה
  של המודול.
- מצב תצורה הוא תמיד 9600 8N1. אם ה-UART רץ במהירות אחרת, הדרייבר פותח מחדש את הפורט לפני
  ואחרי כל קריאת תצורה.
- אם AUX מוחזק LOW בזמן שהמודול נדלק, המודול נכנס למצב עדכון קושחה ומפסיק להגיב. אף פעם לא
  לשים pull-down על AUX.
- ב-30 dBm המודול מושך עד 620mA בזמן שידור. אם ה-5V נופל, המודול מתאפס באמצע פקטה.
- בקושחה 7453-0-21 ומעלה, AUX לא יורד ל-LOW בזמן השידור ברדיו אלא אם `AT+UAUX` מופעל. אז
  `send()` חוזרת כשהבאפר הטורי ריק, לא כשהפקטה שודרה.

## מצב

נבדק על השולחן ב-2026-10-06 עם שני Arduino Uno ושני מודולי E22-400T30D (קושחה 7453-0-21),
הרדיו על SoftwareSerial, עד 21 dBm. מה עבר:

- קריאה וכתיבה של התצורה עם קריאה חוזרת, כל קריאות ה-`set...`, פקודות AT
- כל ששת קצבי האוויר בשני הכיוונים, 120 מתוך 120 פקטות הגיעו
- קצב אוויר, ערוץ או מזהה רשת שונים בשני המודולים: לא מתקבל כלום
- כתובות: כתובת שונה לא מקבלת כלום; שליחה מ-0xFFFF או האזנה על 0xFFFF מקבלת הכול
- `sendTo()` במצב fixed-point: מגיע בלי כותרת 3 הבייטים, כתובת או ערוץ שגויים לא מקבלים כלום
- הודעה של 300 בייט, מחולקת לפקטות של 240 ושל 32 בייט
- במצבי שינה ותצורה לא מתקבל כלום; `send()` מסרבת במצב שינה וב-wake-on-radio
- מהירויות UART של 19200 ו-38400, כולל המעבר ל-9600 בשביל תצורה

עוד לא נבדק: לוח הטיסה עצמו (בלי AUX, SoftwareSerial על D2/D3), HardwareSerial, ה-ESP32, שמירת
התצורה בפלאש (`C0`), timeouts כשהמודול מנותק, קבלה במצב wake-on-radio, listen-before-talk,
relay, והספק שידור מעל 21 dBm. יציאת ה-RF עוד לא נמדדה בנתח ספקטרום.

המודולים הגיעו עם הרגיסטרים `00 00 00 62 00 17 83 00 00`. רגיסטר 6 הוא `0x83`, לא `0x03` כמו
בתיעוד, אז בייט ה-RSSI מופעל מהמפעל.

פריסת הבייטים הושוותה גם מול [הספרייה ל-E22](https://github.com/xreef/EByte_LoRa_E22_Series_Library)
של Renzo Mischianti.

רישיון MIT.

</div>
