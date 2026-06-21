# TomsLoRaSOS

TomsLoRaSOS ist ein Outdoor-Prototyp fuer den LilyGO T-LoRa Pager. Er kombiniert
GPS, Offline-Karte, WLAN-Kachelcache, direkte LoRa-Nachrichten, Positionspakete
und einen experimentellen SOS-/Totmannmodus.

Wichtig: Das Programm ist kein zertifiziertes Notrufsystem. Es ersetzt kein
Mobiltelefon, kein Satelliten-Notrufgeraet und keine professionellen
Sicherheitsverfahren.

## Was das Programm kann

- GPS-Position auf einer Karte anzeigen
- PNG-Kartenkacheln von der SD-Karte lesen
- fehlende sichtbare Kartenkacheln per WLAN nachladen und cachen
- Textnachrichten und Positionsmeldungen direkt per LoRa austauschen
- SOS-Pakete senden und quittieren
- Tracks als GPX auf der SD-Karte speichern
- Display bei Bedarf automatisch ein- und ausschalten

## Voraussetzungen

- LilyGO T-LoRa Pager mit passender Funkvariante
- MicroSD-Karte, FAT32 formatiert
- Arduino IDE oder `arduino-cli`
- LilyGoLib und die von LilyGO benoetigten Bibliotheken
- `PNGdec` von Larry Bank / bitbank2
- Fuer Kacheln per WLAN: WLAN-Zugang und eine erlaubte Tile-URL

Wenn das offizielle LilyGO-Factory-Beispiel mit deinem Board kompiliert, ist die
Basisumgebung normalerweise richtig.

## Hersteller-HAL vorbereiten

Der Sketch nutzt einige Dateien aus dem offiziellen LilyGO-Factory-Beispiel. Das
Vorbereitungsskript kopiert diese Dateien aus deiner installierten LilyGoLib in
den Sketch.

```powershell
cd .\TomsLoRaSOS
py .\prepare_vendor_files.py
```

Falls die Library nicht automatisch gefunden wird:

```powershell
py .\prepare_vendor_files.py --lilygolib "C:\Users\DEINNAME\Documents\Arduino\libraries\LilyGoLib"
```

## SD-Karte vorbereiten

Kopiere den Inhalt von `TomsLoRaSOS/sdcard` in das Root der SD-Karte.
Danach `pager.ini.example` in `pager.ini` umbenennen.

Beispiel:

```ini
device_id=pager01
device_name=Pager 01
wifi_ssid=MeinWLAN
wifi_password=MeinPasswort
tile_url=https://tile.openstreetmap.org/{z}/{x}/{y}.png
fallback_lat=0.0
fallback_lon=0.0
zoom=18
position_interval_seconds=60
safety_no_motion_seconds=30
safety_countdown_seconds=30
auto_display=1
```

Die Ordnerstruktur auf der SD-Karte sieht danach ungefaehr so aus:

```text
/pager.ini
/maps/
/messages/
/tracks/
```

## Kartenkacheln

Die Firmware erwartet Slippy-Map-Kacheln als PNG:

```text
/maps/<zoom>/<x>/<y>.png
```

Beispiel:

```text
/maps/18/137543/89431.png
```

Ja: Wenn WLAN aktiviert ist und eine gueltige `tile_url` in `pager.ini` steht,
kannst du einfach die Karte bewegen. Fehlt eine aktuell sichtbare Kachel, wird
sie geladen und auf der SD-Karte gespeichert. Durch ein bisschen Scrollen in der
Gegend entsteht also nach und nach ein lokaler Cache fuer genau diese Umgebung.
Wenn WLAN erst spaeter eingeschaltet wird, erkennt die Firmware die Verbindung,
zeichnet die Karte neu und fordert die sichtbaren fehlenden Kacheln erneut an.

Bitte fair bleiben: Der oeffentliche OpenStreetMap-Standardserver ist fuer
normale interaktive Nutzung mit Cache gedacht, nicht fuer massenhaftes
Vorabladen. Fuer groessere Offline-Gebiete solltest du einen eigenen Tile-Server
oder einen Anbieter verwenden, der Offline-Downloads ausdruecklich erlaubt.

Fuer erlaubte Offline-Pakete gibt es:

```powershell
cd .\TomsLoRaSOS
py .\tools\tile_packager.py `
  --tile-url "https://DEIN-ERLAUBTER-SERVER/{z}/{x}/{y}.png" `
  --lat 0.0 `
  --lon 0.0 `
  --radius-km 3 `
  --zooms 14 15 16 17 18 19 `
  --out "E:\maps"
```

## Kompilieren und Flashen

Arduino IDE:

1. `TomsLoRaSOS/TomsLoRaSOS.ino` oeffnen.
2. Board `LilyGo T-LoRa-Pager` waehlen.
3. Gleiche Funkvariante und Boardoptionen verwenden wie beim LilyGO-Beispiel.
4. Kompilieren und hochladen.

Mit `arduino-cli`:

```powershell
arduino-cli compile --fqbn esp32:esp32:tlora_pager .\TomsLoRaSOS
arduino-cli upload  --fqbn esp32:esp32:tlora_pager -p COMx .\TomsLoRaSOS
```

## Bedienung

| Eingabe | Funktion |
|---|---|
| Scrollrad drehen | Karte bewegen oder Listen/Tracks auswaehlen |
| Scrollrad kurz druecken | Auswahl bestaetigen oder Track starten/stoppen |
| `m` | Nachrichtenfenster oeffnen/schliessen |
| `n` | Nachrichtenfenster oeffnen und Eingabe fokussieren |
| `k` | Kartenansicht |
| `w` | WLAN-Konfiguration |
| `p` | Peer-/Nachrichtenansicht |
| `s` | SOS-/Safety-Modus umschalten |
| `c` | Karte auf GPS-Position zentrieren |
| `a` | Kartenachse fuer Scrollrad wechseln |
| `t` | Track-Ansicht |
| `1` bis `4` | Zoom 16 bis 19 |
| `+` / `-` | Zoom hinein/heraus |
| Enter | Nachricht senden oder Warnung quittieren |
| Backspace | Zeichen loeschen |

Im WLAN-Popup gibt es vier fokussierbare Felder: Name, SSID, Passwort und
`WLAN einschalten/ausschalten`. Der Scrollrad-Klick wechselt zwischen diesen
Feldern. Enter speichert im Passwortfeld; auf dem WLAN-Feld schaltet Enter WLAN
ein oder aus und schliesst das Popup. `q` oder Esc schliesst das Popup ohne
weitere Aktion.

Die Statuszeile zeigt den SOS-/Totmannmodus explizit als `SOS: Off`, `SOS: On`,
`SOS: Warn` oder `SOS: Send`.

## LoRa-Hinweis

Das Projekt nutzt ein eigenes kompaktes Textprotokoll fuer POS, MSG, SOS, ACK
und SAFE. Meshtastic ist dafuer nicht erforderlich. Verwende Frequenz,
Sendeleistung und Bandbreite nur so, wie sie an deinem Standort erlaubt sind.

## Datenschutz

Tracks, Nachrichten, WLAN-Daten und Kartenkacheln liegen auf deiner SD-Karte
oder im Geraetespeicher. Lade keine persoenlichen Tracks oder echten
Zugangsdaten in ein oeffentliches Repository hoch.

## Lizenz

Der projekt-eigene Code von TomsLoRaSOS ist fuer Open-Source-Nutzung unter
GPL-3.0-or-later verfuegbar. GPL-konforme kommerzielle Nutzung ist erlaubt. Wer
den Code in geschlossene Produkte integrieren oder ohne GPL-Pflichten weitergeben
moechte, braucht eine separate kommerzielle Lizenz.

Vendor-Dateien aus LilyGO-Beispielen und externe Bibliotheken behalten ihre
jeweiligen Lizenzen. Siehe `LICENSE` und `COPYING.GPL-3.0`.

## Autor und Blog

Dieses Projekt ist Teil einer kleinen LilyGO-T-LoRa-Pager- und Outdoor-
Experimentesammlung von Tom. Weitere Notizen, Experimente und Hintergruende
gibt es auf:

```text
https://steinlaus.de/
```

Wenn du TomsLoRaSOS ausprobierst, verbesserst oder als Idee fuer ein eigenes
Projekt nutzt, ist ein Link zum Blog willkommen.
