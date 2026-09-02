# Specyfikacja: pakowanie strumieniowe „w locie" (blokowe LZSS)

Cel: zapisywać od razu **mały plik** podczas nagrywania (bez pakowania po fakcie),
znacząco wydłużając możliwy czas sesji na małej pamięci M5StickS3.

Decyzje oparte na pomiarach na prawdziwych nagraniach (`ses00043` bezruch,
`ses00045` ruch) — patrz sekcja „Uzasadnienie".

---

## Założenia projektowe

1. **Blokowość z resetem słownika** — kompresja per blok (słownik LZSS zerowany na
   początku każdego bloku). Dzięki temu nagła utrata zasilania psuje **tylko ostatni
   niedokończony blok**, a nie cały plik. To kluczowa różnica wobec kompresji ciągłej.
2. **Blok = 64 rekordy** — tyle, ile writerTask już teraz buforuje przed zapisem.
   Kompresja wpina się w istniejący punkt zapisu; brak zmian w logice kolejki/IMU.
3. **Ten sam algorytm LZSS** co istniejący `packFileLzss`/`verifyLzssFile`
   (okno 2047, dopasowanie 3..34 B, grupa 8 decyzji) — kod enkodera/dekodera już
   sprawdzony, round-trip zweryfikowany na danych.
4. **Fallback na literały** — jeśli blok skompresowany wyszedłby ≥ surowego
   (dane nieściśliwe), zapisujemy blok jako surowy (flaga w nagłówku bloku).
   Gwarancja: plik nigdy nie jest większy niż surowy + drobny narzut nagłówków.
5. **Bufory globalne** — okno historii i bufor wejścia LZSS jako zmienne globalne
   (nie na stosie writera), tak jak już zrobiono z `writerRawBlock`/`writerNormalBlock`
   (stos writera 8 KB nie zniesie dodatkowych ~4 KB tablic).
6. **Opcjonalność / kompatybilność** — nowy format ma własny magic i flagę; stare
   pliki i parser pozostają w pełni obsługiwane. Zapis skompresowany włączany trybem
   sesji (osobne komendy START), nie zmienia istniejących ścieżek NORMAL/RAW.

---

## Nagłówek pliku (64 B) — bez zmiany rozmiaru

Ten sam `SessionHeader` (64 B). Rozróżnienie formatu:
- **magic**: `RIMUZ1` (skompresowany NORMAL) / `RRAWZ1` (skompresowany RAW LAB).
- **formatVersion**: `4` (ta sama generacja).
- **recordBytes**: rozmiar rekordu PRZED kompresją (32 lub 48) — parser wie, na jakie
  rekordy rozłożyć rozpakowane dane.
- Nowy bajt flagi w polu dotąd `reservedByte`: `COMPRESSION=1` (LZSS blokowy).
  `reservedByte2` = liczba rekordów na blok (domyślnie 64), by parser nie zgadywał.

Reszta pól (osie, gyroBias, referencje siły) bez zmian.

## Strumień po nagłówku: sekwencja bloków

Każdy blok:
```
[u16 rawLen]   # ile bajtow SUROWYCH danych zawiera blok (<= 64*recordBytes)
[u16 compLen]  # dlugosc pola danych ponizej
[u8  flags]    # bit0: 0=surowy, 1=LZSS
[compLen B]    # dane: albo surowe rawLen B, albo strumien LZSS (grupy 8 decyzji)
```
- Ostatni blok sesji może mieć `rawLen` < pełnego (resztka < 64 rekordów).
- LZSS strumienia bloku: identyczny jak `RZIP01` (bez wewnętrznego nagłówka RZIP —
  długości są w nagłówku bloku). Dekoder = pętla z `verifyLzssFile`, ograniczona do
  `rawLen` bajtów, słownik zerowany na starcie bloku.

## Zakończenie pliku
Brak stopki. Parser czyta bloki aż do EOF. Niekompletny ostatni blok (awaria
zasilania w trakcie zapisu compLen bajtów) → parser odrzuca tylko ten blok,
reszta czytelna.

---

## Uzasadnienie (pomiary)

LZSS blokowy z resetem słownika, algorytm firmware, na prawdziwych danych:

| Blok | Bezruch (ses43) | Ruch (ses45) |
|------|-----------------|--------------|
| 64 rekordy (3072 B) | **32.2%** | **68.2%** |
| 128 rekordów | 30.3% | 67.1% |
| 256 rekordów | 29.4% | 66.6% |

- Zysk kompresji prawie nie rośnie z rozmiarem bloku → wybieramy **64** (najlepsza
  odporność na awarię + minimalny RAM + zgodność z istniejącym buforowaniem writera).
- Realny efekt: aktywna jazda ~68% oryginału (**~1.5× dłuższa sesja**), jazda z
  przerwami/bezruchem znacznie więcej (bezruch pakuje się do ~32%).
- Round-trip enkoder↔dekoder zweryfikowany bajt-w-bajt na obu nagraniach.

## Ryzyka i mitygacje
- **Obciążenie CPU**: enkoder brute-force O(okno) na bloku 3 KB co ~0.32 s, na rdzeniu
  writera (core 0, priorytet 1); IMU (core 1, priorytet 3) nietknięte. Weryfikacja na
  sprzęcie: licznik `qdrop` w DIAG musi zostać 0 przy najgęstszej jeździe.
- **Wzrost RAM**: ~4 KB globalnie (okno 2048 + bufor). Budżet jest (kolejka 24 KB itd.).
- **Kompatybilność**: nowy magic; istniejące formaty i pakowanie po fakcie zostają.
