# Łatka firmware `rolki.ino` — poprawki 2.2, 2.3, 2.5, 2.6 (NAŁOŻONA)

Poprawki są już **nałożone na plik `rolki.ino`** w tym workspace (pobrany z
`github.com/rsmkrsmk/imu`). Gotowy diff: **`rolki.patch.diff`**.

Format zapisu **nie zmienia się** (te same struktury, rozmiary rekordów, ten sam
`formatVersion`) — zmieniają się tylko wartości pól i stałe filtrów, więc istniejący
parser czyta nowe pliki bez modyfikacji.

> **Uwaga o dostarczeniu:** w tej sesji uwierzytelnianie GitHub przez bramkę nie
> działa (`git` i `gh api` zwracają „No user principal found for this instance”),
> więc **nie mogłem wypchnąć zmian ani otworzyć PR**. Odczyt plików działał tylko
> dzięki bezpośredniemu `raw.githubusercontent.com`. Plik `rolki.ino` i `rolki.patch.diff`
> są gotowe w workspace — wystarczy je wypchnąć z Twojej strony (instrukcja niżej).

---

## Podsumowanie kalibracji (zweryfikowane na danych)

Symulacja pełnego potoku stroke na Twoich RAW (`ses00038.rwl`, `ses00040.rwl`)
z **finalnymi stałymi firmware**:

| Sesja        | strokes | kadencja | SS p50 | SS p95 | SS max |
|--------------|---------|----------|--------|--------|--------|
| calm (38)    | 112     | 71/min   | 45     | 57     | 64     |
| dynamic (40) | 23      | 86/min   | 71     | 100    | 100    |

Rozkłady per-stroke przy HP=1.0 Hz (na podstawie których dobrano referencje):

| Wielkość      | calm p50 / p95 | dynamic p50 / p95 |
|---------------|----------------|-------------------|
| peak hpAf [g] | 2.24 / 2.94    | 2.57 / 5.48       |
| surge [g/s]   | 51.6 / 91.8    | 47.5 / 132.5      |
| impulse [g·s] | 0.109 / 0.164  | 0.148 / 0.330     |

Przy HP=20 Hz `hpAf p95` spadało do 0.48 g / 0.91 g — filtr gubił pasmo odepchnięcia
(~2–5 Hz); HP=1 Hz je odzyskuje, a poniżej ~0.5 Hz nie ma już różnicy.

---

## Zmienione stałe

```cpp
HP_CUTOFF_HZ           : 20.0f  -> 1.0f       // 2.5
STROKE_REF_PEAK_G      : 0.55f  -> 5.0f
STROKE_REF_IMPULSE_GS  : 0.05f  -> 0.30f
STROKE_REF_SURGE_GPS   : 45.0f  -> 130.0f
STROKE_ENTER_SURGE_GPS : 6.0f   -> 20.0f
STROKE_ENTER_HP_G      : 0.20f  -> 0.80f
STROKE_CONFIRM_SURGE_GPS: 10.0f -> 40.0f
STROKE_CONFIRM_HP_G    : 0.35f  -> 1.30f
STROKE_REFRACTORY_US   : 280000 -> 450000
```
Wagi bez zmian (0.40 / 0.30 / 0.30, suma = 1.0).

## Zmiany w kodzie `imuTask`

- **2.2** — dodana zmienna `strokeStrengthHeld` trzyma pełną siłę ostatniego
  odepchnięcia (peak+impuls+surge); `record.strokeStrength_x10` liczony z niej,
  a nie z `strengthInstant` (który pomijał składnik impulsu → zaniżenie).
- **2.3** — `record.intensitySmooth_x10` przez `clampU16` (pole bez znaku 0..1000).
- **2.6** — `surge` liczony jako pochodna wygładzonego `af` (`afLp` + `previousAfLp`),
  zamiast `(hpAf - previousHp)/dt` (podwójne różniczkowanie wzmacniało szum);
  usunięto zbędne `previousHp`.

`strengthInstant` pozostaje — nadal zasila strefy B i histogram (chwilowa wartość).

---

## Jak wypchnąć (z Twojej strony)

```bash
git clone https://github.com/rsmkrsmk/imu.git
cd imu
git checkout -b fix/stroke-calib-2.2-2.3-2.5-2.6
# podmień rolki.ino na wersję z tego workspace  (albo: git apply rolki.patch.diff)
git add rolki.ino
git commit -m "Poprawki 2.2/2.3/2.5/2.6 + kalibracja stroke (HP 1Hz, refy na danych)"
git push -u origin fix/stroke-calib-2.2-2.3-2.5-2.6
# potem otwórz PR na GitHub
```

Alternatywnie, jeśli masz oryginał lokalnie: `git apply rolki.patch.diff`.


---

# Uzupełnienie: poprawki 2.1, 2.7, 2.8, 2.9 (NAŁOŻONE)

Nałożone na `rolki.ino` obok 2.2/2.3/2.5/2.6. Rozmiary struktur bez zmian
(64 / 32 / 48 B — `static_assert` nietknięte), więc **format nadal kompatybilny**.

## Poprawka 2.1 — spójność nagłówka

- **`formatVersion` = 4 dla OBU formatów** (było: NORMAL=4, RAW LAB=2 — dwa formaty
  tej samej generacji miały różne numery wersji). Format rozróżnia teraz magic
  (`RIMU04` / `RRAW02`), a `formatVersion` jednoznacznie mówi „generacja v4".
- **Magic kopiowany bezpiecznie** na pełne 8 B z gwarantowanym zerowaniem (było
  `strncpy(..., sizeof-1)` — kruche przy 8-znakowym magicu).
- **Rozmiar rekordu zapisywany dla OBU formatów** — pole `reserved` (u32)
  przemianowane na `recordBytes` i wypełniane `sizeof(SampleRecord)`=32 dla NORMAL
  (było 0) oraz `sizeof(RawLabRecord)`=48 dla RAW LAB. Parser nie musi zgadywać
  rozmiaru rekordu z magica.
- Zaktualizowane mylące komentarze (referencje siły, opis formatu v3/v4).

> Zmieniają się **wartości** bajtów w nagłówku (formatVersion RAW LAB 2→4,
> recordBytes NORMAL 0→32), ale **nie offsety ani rozmiar** — parser czytający pola
> po offsetach działa dalej; warto tylko zaktualizować go, by ufał `recordBytes`.

## Poprawka 2.7 — dryf przechyłu (lean)

Przechył to filtr komplementarny: `relLean += rollRate*dt` (całka żyroskopu)
ściągana ku kątowi z grawitacji wagą `correction`. Przy jeździe `correction`
spadał do 0.004, więc bias żyroskopu akumulował się aż do limitu ±75°.

- `LEAN_BASE_CORRECTION`: 0.004 → **0.010** (mocniejsze ściąganie w ruchu).
- Nowa stała `LEAN_IDLE_DECAY_PER_S = 0.5` — przy bezruchu integrator jest powoli
  ściągany ku kalibracji (kasuje dryf skumulowany w postojach), proporcjonalnie do
  „spoczynku" (`1 − max(linearMotion, angularMotion)`) i `dt`.

## Poprawka 2.8 — czas fazy odepchnięcia

`strokePhaseMs` niósł **czas do szczytu** (`candPeakUs − candStartUs`), mimo że pole
opisano jako „czas trwania impulsu".

- Dodano `candLastActiveUs` — ostatnia chwila, gdy sygnał był powyżej progu wejścia.
- `strokePhaseMs` = **rzeczywisty czas trwania fazy aktywnej** (`candLastActiveUs −
  candStartUs`), clamp podniesiony 150 → 250 ms (mieści się w `uint8_t`).
- Komentarz pola uspójniony.

## Poprawka 2.9 — drobiazgi

- **Literówka klucza API**: `"stokes_per_min"` → `"strokes_per_min"` — poprawione
  **w obu miejscach**: w JSON `/api/status` (backend) **oraz** w odczycie JS
  dashboardu (`s.stokes_per_min` → `s.strokes_per_min`), więc wbudowany front działa
  dalej, a zewnętrzni konsumenci API dostają poprawny klucz.
- **Percentyle P90/P95 liczone tylko dla próbek AKTYWNYCH** (`if (!idle)`) — wcześniej
  histogramy zliczały też bezruch (masowo bin 0), co zaniżało percentyle.
- **Indeks histogramu ograniczony jawnie do 0..100** (`clampFloat(...,0,100)` zamiast
  polegać na rzucie `uint8_t`) — usuwa ukryte ryzyko zapisu poza tablicą [101].

---

## Stan dostarczenia

Wszystkie 8 poprawek (2.1, 2.2, 2.3, 2.5, 2.6, 2.7, 2.8, 2.9) są w `rolki.ino`
i w `rolki.patch.diff`. Push/PR wciąż niemożliwy w tej sesji (brak tożsamości
GitHub w bramce) — instrukcja ręcznego pushu jak wyżej.


---

# Uzupełnienie: przeprojektowanie strony WWW (dashboard telefonu)

Przeprojektowana warstwa wizualna wbudowanego panelu (`PHONE_DASHBOARD_HTML`).
**Wszystkie `id`, endpointy API i funkcje JS zachowane** — zero zmian w danych/logice,
tylko wygląd, hierarchia i drobne poprawki treści.

## Cele (wg zamówienia: 1 strona, atrakcyjnie, czytelnie podczas jazdy)
- **Dominująca karta SIŁY** (hero) z ogromną liczbą, paskiem `gauge` i etykietą strefy —
  czytelna z odległości rzutem oka podczas jazdy.
- **Kolor akcentu całej karty zmienia się wg aktualnej strefy** siły (biały→cyjan→żółty→
  pomarańcz→czerwień) — natychmiastowy sygnał intensywności bez czytania liczb.
- **KADENCJA + MOC** jako czytelna para tuż pod hero; reszta metryk niżej, uporządkowana.
- Większa, „przyklejona" (sticky) nawigacja z 6 widokami; delikatne animacje wejścia.
- Odświeżona paleta (głębsze tło, gradienty akcentowe), spójne karty/wykresy, lepszy kontrast.

## Zmiany techniczne
- Przepisany blok `<style>` (zmienne `--z0..--z4`, `--accent`/`--accentSoft`, hero `.bigCard`,
  `.gauge`, `.duo`; ujednolicone karty, taby, wykresy, dock akcji).
- Przebudowany układ widoku LIVE (kolejność sekcji wg ważności).
- `updateZones()` ustawia `--accent`/`--accentSoft` i szerokość `#strengthGauge` wg strefy.
- Poprawki treści/jednostek: „czaz impulsu"→„czas trwania impulsu", usunięte **podwójne
  jednostki** (`g/s`, `ms` — jednostka pochodzi teraz z elementu HTML, nie z JS).

## Weryfikacja
- Podgląd w przeglądarce (mobilny viewport 402 px): hero, gauge, kolor strefy, nawigacja
  i wykres RYTM renderują się poprawnie i czytelnie.
- Brak sekwencji `)HTML"` w treści (literał C++ nienaruszony); wszystkie `id` używane w JS
  mają swój element; znaczniki zbalansowane (div 96, span 91, article 17, section 6, button 20).
- 8 poprawek firmware (2.1–2.9) nienaruszonych. CRLF zachowane.
