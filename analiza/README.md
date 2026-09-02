# Analiza i kalibracja firmware `rolki.ino`

Ten katalog zawiera materiały pomocnicze użyte przy analizie kodu, weryfikacji
algorytmów i kalibracji poprawek 2.1–2.9. **Nie są częścią firmware** — służą do
analizy nagrań i doboru stałych na PC (Python 3).

## Dokumentacja
- **`PATCH_rolki_firmware.md`** — pełny opis wszystkich poprawek (2.1–2.9) +
  przeprojektowania dashboardu, z uzasadnieniem i wynikami kalibracji na danych.
- **`rolki.patch.diff`** — zbiorczy diff wszystkich zmian względem oryginału
  (podgląd całości w jednym miejscu; historia krok po kroku jest w commitach).

## Skrypty (kolejność użycia)
1. **`inspect.py`** — wykrywa format pliku (magic RIMU03/04, RRAW01/02), rozmiar
   nagłówka i rekordu.
2. **`probe_raw.py`** — empirycznie wyprowadza układ rekordu RAW v1 (40 B),
   potwierdza mapowanie osi (FORWARD=X, VERTICAL=Y, LATERAL=Z) i skalę.
3. **`parse_v3.py`** — parser RAW + funkcje statystyczne (percentyle, dt, |acc|,
   surge, siła). Importowany przez pozostałe skrypty.
4. **`measure.py`** — mierzy rozkłady per-odepchnięcie (peak/surge/impuls) przy
   HP=1 Hz, na podstawie których dobrano referencje siły.
5. **`sim_hp.py`** — symuluje pełny potok stroke dla różnych częstotliwości HP
   (20/5/2/1/0.5/0.3 Hz); pokazał, że HP=20 Hz gubi pasmo odepchnięcia.
6. **`calib.py`** — dobór referencji i progów detektora tak, by siła rozkładała
   się 0–100, a kadencja per-łyżwa była realistyczna (~70/min).

## Jak uruchomić (przykład)
```bash
cd analiza
python3 sim_hp.py ../ses00038.rwl ../ses00040.rwl
python3 measure.py            # (ścieżki do .rwl ustawione w skrypcie)
```

> Uwaga: pliki `ses00038.rwl` / `ses00040.rwl` w repo to nagrania ze STAREGO
> firmware (format RRAW01/RIMU03). Do weryfikacji poprawek 2.1/2.7/2.8 potrzebne
> są nowe nagrania z firmware po zmianach (RRAW02, formatVersion 4).
