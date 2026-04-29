# 🤖 Referencja Pipeline ML — `simulation/ml/`

To jest referencja moduł po module dla pipeline'u ML detekcji SAR.
Praktyczny tutorial znajdziesz w [`ml_tutorial.pl.md`](ml_tutorial.pl.md).
Opis ciężkich komponentów (prawdziwy trening PyTorch, eksport ONNX/TensorRT, 
wdrożenie na Jetson, przechwytywanie świata SAR w Gazebo) znajduje się w 
[`nightly_lane.md`](nightly_lane.md).

---

## Architektura w skrócie

Pakiet dostarcza **dwie równoległe warstwy** o tych samych konwencjach 
(rejestr, bramka KPI, logika promocji):

* **Detekcja (Detection)** — optymalizuje model *percepcji*.
* **Sterowanie (Control)** — optymalizuje wzmocnienia *kaskadowego kontrolera PID* 
  dla pojedynczego drona realizującego punkty nawigacyjne (waypoints).

```
DETEKCJA                          │   STEROWANIE
                                  │
┌──────────────────────┐          │   ┌────────────────────────┐
│  sar_targets.py      │          │   │  missions.py           │
│  katalog 21 klas     │          │   │  4 rodzaje misji       │
└──────────┬───────────┘          │   └──────────┬─────────────┘
           │                      │              │
┌──────────▼─────────┐            │   ┌──────────▼─────────────┐
│ coco_annotator +   │            │   │ waypoint_optimizer.py  │
│ image_augment +    │            │   │ run_episode +          │
│ model_zoo (6 back) │            │   │ random_search          │
└──────────┬─────────┘            │   │ po PolicyGains         │
           │                      │   └──────────┬─────────────┘
┌──────────▼─────────┐            │              │
│ inference_logger   │            │   ┌──────────▼─────────────┐
│ JSONL na dysku     │            │   │ waypoint_kpi.py        │
└──────┬─────────────┘            │   │ evaluate +             │
       │                          │   │ compare_waypoint_      │
┌──────▼──────┐ ┌─────────────┐   │   │ policies               │
│ hard_example│ │ kpi +       │   │   └──────────┬─────────────┘
│ _miner      │ │ compare_    │   │              │
└─────────────┘ │ models      │   │              ▼
                └──────┬──────┘   │   ┌──────────────────────┐
                       │          │   │ model_registry       │
                       └──────────┴──►│ (wspólny — jeden     │
                                      │ plik JSON, lineage)  │
                                      └──────────────────────┘
```

Każdy moduł to **czysty Python** — bez PyTorcha, bez Ultralytics, bez OpenCV.
Kontrakt z odroczonymi backendami (`yolo`, `rtdetr`, `detr`, `fcos`, `onnx`, 
`tensorrt`) jest jawną rejestracją z wyraźnym błędem `NotImplementedError` 
wskazującym na "nightly lane".

---

## 1. `sar_targets.py` — Katalog Celów

Pojedyncze źródło prawdy (SSoT) dla 21 klas detekcji SAR. Każdy wpis zawiera 
swoje COCO `category_id`, nazwę, superkategorię i fizyczne wymiary 
`(length_m, width_m, height_m)` — których adnotator COCO używa do wymiarowania 
syntetycznych ramek otaczających (bounding boxes).

### Schemat

```python
@dataclass(frozen=True)
class SARTarget:
    category_id: int          # COCO ID (1-9 zgodne z COCO 2017, 100+ SAR)
    name: str
    supercategory: str        # person | vehicle | equipment | hazard
    footprint_m: tuple        # (length, width, height) w metrach
    canonical_yaw: float = 0.0
```

### Katalog (21 klas)

| Zakres | Klasa | Zastosowanie |
|:--|:--|:--|
| 1-9    | person, bicycle, car, motorcycle, bus, truck, boat | Transfer-learning zgodny z COCO 2017 |
| 100-104 | casualty, stretcher, tent, tarp, medkit | Główne cele SAR |
| 105-109 | drone, fire, smoke_plume, debris, vehicle_wreck | Operacyjne + znaczniki zagrożeń |
| 110-113 | raft, lifevest, flare, supply_drop | Morskie + zrzuty SAR |

### API

```python
from ml.sar_targets import find, coco_categories, total_count, SAR_TARGETS

person = find("person")               # SARTarget(category_id=1, ...)
cats   = coco_categories()            # [{id, name, supercategory}, ...]
n      = total_count()                # 21 (obecny rozmiar katalogu)
```

### Inwarianty wymuszane przez testy

- Wszystkie 21 klas jest obecnych (`>= 20` dla akceptacji katalogu).
- ID kategorii są unikalne.
- Superkategorie obejmują `{person, vehicle, equipment, hazard}`.
- Każdy wymiar (footprint) jest 3-elementową krotką ściśle dodatnich metrów.
- `find()` rzuca `KeyError` z listą dostępnych klas w przypadku braku trafienia.
- `SARTarget` jest "zamrożony" (frozen) — wpisów nie można zmieniać w runtime.

---

## 2. `coco_annotator.py` — Projekcja Pinhole → COCO JSON

Przekształca syntetyczne klatki (nazwa pliku obrazu + poza drona + pozy celów) 
w słownik `dataset` formatu COCO 2017, gotowy dla `pycocotools` lub dowolnego 
trenera detekcji obsługującego COCO.

### Model kamery

```python
@dataclass
class CameraIntrinsics:
    fx: float; fy: float        # ogniskowe w pikselach
    cx: float; cy: float        # punkt główny w pikselach
    width: int; height: int     # rozmiar obrazu w pikselach
```

Projekcja zakłada **kamerę skierowaną w dół** (typowe dla SAR). Dla każdego celu 
przesunięcie `(dx, dy, dz)` między dronem a celem jest rzutowane za pomocą 
trójkątów podobnych:

```
u  = cx + fx · dx / dz
v  = cy − fy · dy / dz       # negacja y, aby dopasować do współrzędnych obrazu
w_px = fx · footprint_w / dz
h_px = fy · footprint_l / dz
```

### Filtr widoczności

Cel jest usuwany z `annotations`, gdy:

- `dz <= 0` — kamera poniżej lub na poziomie celu.
- Ramka po wymiarowaniu wypada całkowicie poza obraz (`x + w_px <= 0`, 
  `y + h_px <= 0`, `x >= width` lub `y >= height`).
- Przycięta szerokość lub wysokość spada do ≤ 0.

Częściowo widoczny cel jest **przycinany** (clipping) do granic obrazu 
(zachowany ze zredukowaną ramką), a nie usuwany — trener nadal otrzymuje 
częściową prawdę (ground truth).

### Schemat wyjściowy (COCO 2017)

```json
{
  "info":        {"description": "...", "year": 2026, "version": "1.0"},
  "images":      [{"id", "file_name", "width", "height"}],
  "annotations": [{"id", "image_id", "category_id", "bbox": [x,y,w,h],
                   "area", "iscrowd": 0}],
  "categories":  [{"id", "name", "supercategory"}]
}
```

`annotations[].id` rośnie monotonicznie we wszystkich klatkach, zaczynając od 1.

### Zapis na dysk

```python
write(dataset, Path("datasets/sar/train.json"))
```

`write()` tworzy drzewo katalogów nadrzędnych, jeśli nie istnieje, i generuje 
JSON z wcięciem 2 spacji i znakiem nowej linii na końcu.

---

## 3. `image_augment.py` — Randomizacja Domeny (Pillow)

Warstwa augmentacji oparta na Pillow. Wersja produkcyjna zostanie zamieniona na 
Albumentations, gdy wejdzie ciężka gałąź ML, ale ten moduł jest tym, co 
wykonuje CI.

### Specyfikacja

```python
@dataclass(frozen=True)
class AugmentationSpec:
    brightness_low: float = 0.6
    brightness_high: float = 1.4
    contrast_low: float = 0.7
    contrast_high: float = 1.3
    blur_radius_max: float = 1.5
    cutout_count_max: int = 3
    cutout_size_frac_max: float = 0.15
    weather_drop_density: float = 0.002    # gęstość deszczu/śniegu
```

### `Augmenter`

Każda metoda jest **deterministyczna przy użyciu ziarna (seed)**:

```python
aug = Augmenter(spec=AugmentationSpec(), seed=42)
img = aug.random_brightness(img)
img = aug.random_contrast(img)
img = aug.random_blur(img)
img = aug.random_cutout(img)
img = aug.add_weather(img, kind="rain")    # lub "snow"
img = aug.apply_random_chain(img, p=0.5)   # aplikuj każdy krok z prawd. p
```

`add_weather("rain")` rysuje 3-pikselowe pionowe kreski; `"snow"` rozrzuca 
pojedyncze białe piksele. Dowolny inny `kind` rzuca `ValueError`.

`apply_random_chain` uruchamia pięć kroków w ustalonej kolejności, każdy 
warunkowany tym samym prawdopodobieństwem `p`. Ustawienie `p=0.0` to 
identyczność; `p=1.0` nakłada każdą augmentację (używane przez testy 
integracyjne).

---

## 4. `model_zoo.py` — ABC Detektora + Rejestr

Zastępuje twardo zakodowane wywołania `YOLO('yolov8n.pt')` pojedynczym 
interfejsem `Detector`, dzięki czemu wywołujący nie zależą od konkretnego 
backendu.

### Rekord detekcji

```python
@dataclass(frozen=True)
class Detection:
    category_id: int
    category_name: str
    bbox: tuple              # (x, y, w, h) w pikselach — COCO xywh
    confidence: float
    def to_dict(self) -> dict   # bezpieczne dla JSON
```

### ABC Detektora

```python
class Detector(ABC):
    backend: str = "abstract"
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]: ...
    def name(self) -> str: return self.backend
```

### Rejestr

| Backend     | Status   | Wymagana zależność |
|:------------|:---------|:-------------------|
| `mock`      | ✅ aktywny| brak               |
| `yolo`      | ⏸ stub   | `ultralytics`      |
| `rtdetr`    | ⏸ stub   | `ultralytics`      |
| `detr`      | ⏸ stub   | `transformers`     |
| `fcos`      | ⏸ stub   | `torchvision`      |
| `onnx`      | ⏸ stub   | `onnxruntime`      |
| `tensorrt`  | ⏸ stub   | `tensorrt`         |

Każdy "stub" rzuca `NotImplementedError` z nazwą brakującej zależności i 
wskazaniem na `docs/nightly_lane.md`. Powierzchnia API ModelZoo jest więc 
kompletna *dzisiaj* — kod wywołujący nigdy nie musi specjalnie traktować 
przypadku "backend niedostępny", a przejście z `mock` na `yolo` po wejściu 
odpowiedniej gałęzi to zmiana jednej linii w miejscu wywołania.

### Niestandardowa rejestracja

Zewnętrzny kod może rejestrować własne backendy:

```python
from ml.model_zoo import register, ModelZoo
register("my_backend", lambda weights_path=None, **kw: MyDetector(weights_path))
det = ModelZoo().load("my_backend", weights_path="my.pt")
```

Loadery otrzymują `weights_path` oraz dowolne `**kwargs`, dzięki czemu mogą 
przyjmować tuning specyficzny dla backendu (próg pewności, IoU NMS, 
half precision itp.).

### `MockDetector`

Zwraca albo dostarczoną przez wywołującego stałą listę, **albo** domyślną 
listę dwóch detekcji (person + casualty). Przekazanie jawnie `fixed=[]` daje 
zero detekcji — rozróżnienie między pustą listą a `None` ma znaczenie dla 
testów integracyjnych "hard-example miner".

---

## 5. `inference_logger.py` — Przechwytywanie JSONL

Przechwytuje każdy wynik inferencji do pliku JSON Lines, aby hard-example miner 
i bramka promocji mogły odtworzyć zachowanie modelu bez ponownego jego 
uruchamiania.

### Schemat rekordu

```python
@dataclass
class InferenceRecord:
    frame_id: int
    t_wall_s: float
    model_version: str
    image_path: Optional[str] = None
    metadata: dict = field(default_factory=dict)
    detections: List[Detection] = field(default_factory=list)
```

### Cykl życia loggera

```python
with InferenceLogger(Path("logs/run.jsonl"), model_version="v1") as log:
    rec = log.log(detections,
                  image_path="frame_001.png",
                  metadata={"weather": "rain", "expected_targets": 2})
```

- Plik jest otwierany w **trybie dopisywania (append)**, więc ponowne otwarcie 
  częściowego logu poprawnie kontynuuje numerację.
- `_tail_max_frame_id()` skanuje istniejący plik i kontynuuje od 
  `max(frame_id) + 1`. **Uszkodzone linie JSON są pomijane**, więc 
  na wpół zapisany rekord z przerwanego biegu nie psuje następnego.
- `log()` wykonuje flush po każdym zapisie — `KeyboardInterrupt` w trakcie 
  misji powoduje utratę co najwyżej jednego rekordu.

### Odczyt wsteczny

```python
from ml.inference_logger import read_log, replay
records = list(read_log(Path("logs/run.jsonl")))   # strumieniowo
records = replay(read_log(...))                     # zmaterializowana lista
```

`InferenceRecord.from_dict()` toleruje brak opcjonalnych pól (`model_version`, 
`image_path`, `metadata`, `detections`) — stare logi pozostają czytelne 
w miarę ewolucji schematu.

---

## 6. `hard_example_miner.py` — Triaż do Ponownego Etykietowania

Odczytuje logi inferencji, zwraca rekordy najbardziej warte ponownego 
etykietowania dla następnej epoki treningowej.

### Dwie heurystyki

| Powód | Wyzwalacz | Wynik (Score) |
|:-------------|:---------------------------------------------------------|:--------------------------------------------|
| `uncertain`  | Przynajmniej jedna detekcja w przedziale `[low_conf, high_conf]` | `1.0` w centrum pasma, spada ku 0 na brzegach |
| `missed`     | Zero detekcji ORAZ `metadata["expected_targets"] >= 1` | `expected_targets` (surowa liczba) |

```python
@dataclass(frozen=True)
class HardExample:
    frame_id: int
    image_path: Optional[str]
    reason: str           # "uncertain" | "missed"
    score: float          # wyższy = ważniejszy do ponownego etykietowania
    def to_dict(self) -> dict
```

### Konfiguracja

```python
@dataclass
class MinerConfig:
    low_conf: float = 0.30
    high_conf: float = 0.60
    expected_targets_key: str = "expected_targets"
```

### API

```python
mine(records, cfg=MinerConfig())   # obie heurystyki, posortowane po score desc
find_uncertain(records, cfg)       # tylko "uncertain"
find_missed(records, cfg)          # tylko "missed"
```

`mine()` wywołuje oba findery, a następnie sortuje wyniki malejąco według 
wyniku, aby kolejka operatora (CVAT, Label Studio) otrzymała najpierw klatki 
o najwyższej wartości.

### Przypadki brzegowe (z testów)

- Pusta lista rekordów → pusty wynik.
- Klatka z detekcjami nigdy nie jest rejestrowana jako "missed", nawet jeśli 
  `expected_targets > 0` — model coś zwrócił, a przypadek niskiej pewności 
  obsługuje heurystyka `uncertain`.
- Brak metadanych `expected_targets` domyślnie przyjmuje 0 → brak sygnału missed.

---

## 7. `model_registry.py` — Lineage + Stan Promocji

Plik JSON typu append-only śledzący każdą zarejestrowaną wersję modelu i jej 
KPI.

### Format pliku

```json
{
  "models": [
    {
      "version": "yolov8s_sar_v1",
      "backend": "yolo",
      "weights_path": "models/yolov8s_sar_v1.pt",
      "trained_at_s": 1745251200.0,
      "parent_version": null,
      "kpis": {"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
      "notes": "First fine-tune on synthetic SAR data."
    }
  ]
}
```

### API

```python
reg = ModelRegistry(Path("models/registry.json"))
reg.register(ModelEntry(version="v1", backend="yolo",
                         weights_path="...", kpis={...}))
reg.all()                       # list[ModelEntry] (niezależna kopia)
reg.get("v1")                   # KeyError w przypadku braku
reg.latest()                    # ostatnio zarejestrowany lub None
reg.best_by_kpi("mAP_50")       # max po nazwanym kluczu KPI lub None
reg.lineage("v3")               # idzie po wskaźnikach parent_version do korzenia
```

### Gwarancje bezpieczeństwa

- `register()` odrzuca duplikaty ciągów `version` (`ValueError`).
- `lineage()` broni się przed ręcznie edytowanymi cyklami w pliku JSON — używa 
  zbioru `seen` i rzuca `ValueError("cycle detected")` zamiast pętli 
  nieskończonej.
- `lineage()` rzuca `KeyError`, jeśli wersja startowa nie istnieje.
- `register()` **nie** waliduje, czy `parent_version` wskazuje na istniejący 
  wpis — uszkodzone wskaźniki ujawniają się dopiero podczas przechodzenia 
  `lineage()`. Jest to zamierzone: przepływy masowego importu mogą rejestrować 
  dzieci przed rodzicami.
- `all()` zwraca kopię obronną, więc wywołujący nie mogą zmieniać wewnętrznego 
  stanu.
- `_flush()` nadpisuje cały plik przy każdym `register()` — akceptowalne przy 
  obecnej skali (dziesiątki modeli). Produkcja zastąpi to formatem liniowym 
  append-only, jeśli rejestr przekroczy ~10⁴ wpisów.

---

## 8. `kpi.py` — Progi Akceptacji + Bramka Promocji

Liczby, które kandydat na model musi osiągnąć, zanim będzie mógł zastąpić 
obecny model (incumbent).

### Progi akceptacji (bezwzględne minimum)

```python
ACCEPTANCE_THRESHOLDS = {
    "mAP_50":       0.75,    # główna metryka detekcji
    "recall":       0.85,    # nie pomijaj ofiar (casualties)
    "inference_ms": 50.0,    # budżet na klatkę na Jetson Orin Nano
}
```

Źródło: `todo/ml_training_pipeline.md`. Model niespełniający któregokolwiek z 
tych progów jest odrzucany od razu, niezależnie od tego, jak wypada w 
porównaniu z obecnym modelem.

### Delty promocji (relatywna poprawa)

```python
PROMOTION_MIN_DELTA = {
    "mAP_50":       0.005,    # kandydat musi być lepszy o ≥ 0.005 mAP
    "recall":       0.005,    # śledzone, obecnie niewymuszane (patrz poniżej)
    "inference_ms": -1.0,     # latencja: kandydat musi być o ≥ 1 ms szybszy
                              # LUB nie pogorszyć się o więcej niż 0.5 ms
}
```

### `evaluate_kpis(measured: dict) -> DetectionKPIs`

Nakłada bezwzględne progi i zwraca:

```python
@dataclass
class DetectionKPIs:
    mAP_50: float
    recall: float
    inference_ms: float
    extra: Dict[str, float]      # dodatkowe zmierzone metryki
    failures: List[str]          # czytelne dla człowieka opisy błędów
    verdict: str                 # "PASS" | "FAIL"
```

Brakujące klucze są traktowane jako 0.0 (mAP/recall) lub `+inf` (latencja) — 
pusty słownik zawodzi na każdym progu.

### `compare_models(candidate, incumbent) -> (promote: bool, reason: str)`

| Krok | Zachowanie |
|:--|:--|
| 1. Bramka akceptacji | Kandydat musi najpierw przejść `evaluate_kpis()`. |
| 2. Brak obecnego modelu | Jeśli `incumbent is None`, pierwszy akceptowalny model promuje. |
| 3. Regresja latencji | Latencja kandydata nie może przekroczyć obecnego o więcej niż 0.5 ms. |
| 4. Delta głównego KPI | `mAP_50` kandydata musi pobić obecny o ≥ `PROMOTION_MIN_DELTA["mAP_50"]`. |

**Znane ograniczenie:** `compare_models` wymusza bezwzględne minimum recall 
poprzez `evaluate_kpis`, ale nie stawia warunku na *deltę* recall. Kandydat z 
`+0.01 mAP_50` i `−0.04 recall` zostanie promowany, dopóki nadal spełnia 
`recall >= 0.85`. Jest to zamierzone — głównym KPI jest mAP — ale stała 
`PROMOTION_MIN_DELTA["recall"]` jest zarezerwowana na wypadek, gdyby 
wywołujący chcieli wymusić surowszą zasadę.

---

## 9. `waypoint_optimizer.py` — Tuning PID Pojedynczego Drona

Odpowiednik `model_zoo` + `inference_logger` po stronie sterowania dla jednego 
drona. Optymalizuje kaskadowe **wzmocnienia PID pozycji + orientacji**, aby 
dron realizował punkty misji z wysoką skutecznością, dokładnym śledzeniem, 
szybkim ustalaniem i umiarkowanym zużyciem energii.

### Wektor wzmocnień

```python
@dataclass(frozen=True)
class PolicyGains:
    pos_x_kp: float; pos_x_ki: float; pos_x_kd: float
    pos_y_kp: float; pos_y_ki: float; pos_y_kd: float
    pos_z_kp: float; pos_z_ki: float; pos_z_kd: float
    att_roll_kp:  float; att_roll_ki:  float; att_roll_kd:  float
    att_pitch_kp: float; att_pitch_ki: float; att_pitch_kd: float
    att_yaw_kp:   float; att_yaw_ki:   float; att_yaw_kd:   float

PolicyGains.from_baseline()                # domyślne produkcyjne
gains.to_dict() / PolicyGains.from_dict(d) # runda przez JSON
gains.apply_to(controller)                 # mutuje PositionController w miejscu
```

`from_baseline()` zwraca dokładnie to samo ziarno sześciu PID-ów, którego 
używa `drone_physics.PositionController.__init__`, więc "wytrenowana" 
polityka zaczyna od znanego, dobrego punktu.

### Runner epizodów

```python
metrics = run_episode(gains, mission_kind="patrol",
                      params=DroneParams(),
                      dt=0.01, waypoint_radius=0.5,
                      hover_time=1.0, max_time=60.0,
                      wind=None, terrain=None)
```

`mission_kind` to jeden z czterech rodzajów z `simulation/missions.py` 
(`patrol`, `lawnmower`, `escort`, `heavy_lift`); runner wybiera 
`build_mission(kind, n=1)[1]`, więc dron podąża tą samą listą punktów, co w 
scenariuszu na żywo. Zwraca:

```python
@dataclass
class EpisodeMetrics:
    waypoints_reached:    int
    waypoint_count:       int
    rmse_xyz_m:           float    # mierzone tylko w fazie ustalonej (w promieniu)
    time_to_first_wp_s:   float    # +inf jeśli nigdy nie osiągnięto
    total_time_s:         float
    energy_proxy_j:       float    # ∫ thrust dt
    max_overshoot_m:      float    # szczytowe wychylenie poza strefę przechwycenia
    finite:               bool     # False jeśli wektor wzmocnień spowodował rozbieżność
    @property
    def completion_ratio(self) -> float
    def as_kpi_dict(self) -> Dict[str, float]
```

**Settled-RMSE** jest kluczowym wyborem projektowym — uśrednianie błędu 
śledzenia na całym przelocie między odległymi punktami dawałoby gigantyczne 
liczby podyktowane geometrią misji, a nie jakością kontrolera. Liczenie tylko 
próbek "w promieniu" (in-radius) izoluje to, "jak mocno kontroler trzyma punkt, 
gdy już tam jest".

### Ewaluacja wielomisji

```python
agg = evaluate_policy(gains,
                      mission_kinds_to_run=["patrol", "lawnmower"],
                      n_episodes_per_mission=1,
                      seed=0,            # zarezerwowane (brak stochastycznego wiatru dzisiaj)
                      dt=0.01, max_time=60.0)
```

Zwraca słownik konsumowany przez `waypoint_kpi.evaluate_waypoint_kpis()`: 
`completion_ratio`, `rmse_xyz_m`, `time_to_first_wp_s`, `energy_proxy_j`, 
`max_overshoot_m` oraz `finite_ratio` (ułamek epizodów, które nie były 
rozbieżne).

### Trener Random Search

```python
result = random_search(n_trials=16, seed=0,
                       baseline=PolicyGains.from_baseline(),
                       bounds=SearchBounds(pos_lo=0.5, pos_hi=1.5,
                                           att_lo=0.5, att_hi=1.5),
                       mission_kinds_to_run=["patrol"],
                       max_time=60.0)

result.best_gains      # PolicyGains
result.best_metrics    # dict
result.best_objective  # float (wyższy = lepszy)
result.trials          # lista (gains, metrics, obj) — indeks 0 to baseline
```

Cel (objective) to `100 * completion - rmse - 0.001 * energy`; próby 
rozbieżne zwracają `-inf` i są zawsze odrzucane. Próba 0 to *zawsze* 
baseline, więc zwrócony najlepszy wynik gwarantuje jakość co najmniej taką, 
jak domyślne ustawienia produkcyjne.

### Co jest odroczone

Prawdziwi trenerzy RL (PPO/SAC), CMA-ES, optymalizacja bayesowska i tuning 
PX4 SITL in-the-loop — wszystko to podłącza się pod ten sam interfejs 
`PolicyGains`. Zobacz [`nightly_lane.md`](nightly_lane.md). Dzisiejszy 
`random_search` to trener dostarczalny w CI, służący do walidacji interfejsu.

---

## 10. `waypoint_kpi.py` — Akceptacja + Promocja Polityk

Taki sam kształt jak `kpi.py`, ale dla jakości sterowania.

### Progi akceptacji (bezwzględne minimum)

```python
WAYPOINT_ACCEPTANCE_THRESHOLDS = {
    "completion_ratio":     0.95,    # co najmniej 95% punktów osiągniętych
    "rmse_xyz_m":           1.0,     # błąd śledzenia fazy ustalonej ≤ 1 m
    "time_to_first_wp_s":   30.0,    # pierwszy punkt w ciągu 30 s
    "max_overshoot_m":      6.0,     # przeregulowanie poza strefę ≤ 6 m
}
```

Skalibrowane tak, aby domyślne wzmocnienia produkcyjne przechodziły na 
`patrol` i `lawnmower` (40-metrowy przelot w lawnmower ma naturalną drogę 
hamowania do ~6 m).

### Delty promocji

```python
WAYPOINT_PROMOTION_DELTA = {
    "completion_ratio":     0.01,    # +1 punkt procentowy zysku
    "rmse_xyz_m":          -0.05,    # 5 cm dokładniej
    "time_to_first_wp_s":  -0.5,     # 0.5 s szybciej
    "energy_proxy_j":       0.0,     # nie pogarszać
}
```

### Logika promocji (`compare_waypoint_policies`)

| Krok | Zachowanie |
|:--|:--|
| 1 | Kandydat musi przejść `evaluate_waypoint_kpis` (bramka akceptacji). |
| 2 | Jeśli brak obecnej polityki, pierwsza akceptowalna promuje. |
| 3 | Energia nie może się pogorszyć (`Δenergy ≤ 0`). |
| 4 | `completion_ratio` musi poprawić się o ≥ 0.01, LUB — jeśli completion jest takie samo — RMSE musi się poprawić o ≥ 0.05 m. |

Mechanizm "fallbacku" przy identycznym completion pozwala na promocję polityki 
"te same trafienia, ale lepsze trzymanie", bez wymyślania sztucznej delty 
skuteczności.

---

## Podsumowanie Pokrycia Testami

`simulation/test_ml/` zawiera **150 testów w 12 plikach**:

| Plik | Testy | Zakres |
|:--|:--:|:--|
| `test_sar_targets.py`            | 10 | Inwarianty katalogu, frozen dataclass, mapowanie COCO |
| `test_coco_annotator.py`         | 10 | Widoczność projekcji, ID wieloklatkowe, clipping, puste listy, efekty uboczne zapisu |
| `test_image_augment.py`          | 13 | Powtarzalność z ziarnem, każda augmentacja, rodzaje pogody, identyczność przy p=0 |
| `test_model_zoo.py`              | 17 | Zgodność z ABC, rejestr, niestandardowe backendy, przekazywanie kwargs, wszystkie 6 stubów |
| `test_inference_logger.py`       |  9 | Runda przez JSONL, kontynuacja append, tolerancja uszkodzeń, domyślne pola |
| `test_hard_example_miner.py`     | 11 | Obie heurystyki, sortowanie score, przypadki brzegowe, `to_dict` |
| `test_model_registry.py`         | 13 | Persystencja, odrzucanie duplikatów, lineage, detekcja cykli, kopie obronne |
| `test_kpi.py`                    | 16 | Stałe progowe, ścieżki błędów, bramka promocji, akceptacja dokładnej delty |
| `test_pipeline_integration.py`   |  7 | Przepływy między modułami: adnotacja→zapis, augmentacja+adnotacja, inferencja→mine, rejestr+kpi |
| `test_waypoint_optimizer.py`     | 18 | Runda PolicyGains + apply, determinizm epizodów, baseline przechodzi patrol/lawnmower, obsługa rozbieżności, uśrednianie, granice wyszukiwania |
| `test_waypoint_kpi.py`           | 17 | Stałe progowe, ścieżki błędów, bramka promocji (regresja energii, fallback RMSE przy remisie completion) |

Uruchomienie podzbioru:

```bash
.venv/bin/python -m pytest simulation/test_ml/ -q
```

---

## Skrypty Sterujące

Trzy punkty wejścia shell znajdują się w `scripts/`:

| Skrypt | Cel |
|:--|:--|
| `scripts/ml_run_pipeline.sh`        | Demo end-to-end detekcji SAR: syntetyczny dataset → augmentacja → logowanie inferencji → mine → rejestracja. Zapisuje artefakty w `reports/ml_pipeline/<ts>/`. |
| `scripts/ml_train_waypoint.sh`      | Random search po wzmocnieniach PID dla jednego drona. Promuje przez `compare_waypoint_policies` i zapisuje `reports/ml_waypoint/<ts>/policy_registry.json`. |
| `scripts/ml_evaluate_waypoint.sh`   | Ładuje zarejestrowaną politykę i ponownie uruchamia `evaluate_policy()` na misjach, wypisuje podział per-misja + zagregowany werdykt. |

Wszystkie trzy używają venv projektu w `.venv/` — zainicjalizuj go raz przez 
`./run_scenario.sh`, jeśli jeszcze nie istnieje.

### Latanie z wytrenowaną polityką w `--physics-live`

Gdy polityka jest zarejestrowana, przeglądarka na żywo dla pojedynczego drona 
może ją załadować bezpośrednio:

```bash
./run_scenario.sh --physics-live --policy=pid_v1
./run_scenario.sh --physics-live --policy=pid_v2 \
    --policy-registry=/path/to/policy_registry.json
```

Mechanizm:

* `run_scenario.sh` parsuje `--policy=...` i `--policy-registry=...` 
  i przekazuje je jako `--policy` / `--policy-registry` do 
  `physics_live_replay`.
* `physics_live_replay.load_policy_gains(registry_path, version)` 
  rozwiązuje wpis (według wersji lub najlepszy po completion, gdy pominięto) 
  i odtwarza `PolicyGains` z dołączonych wzmocnień — spadając na plik JSON w 
  `entry.weights_path`, gdy brakuje paczki w rejestrze.
* Zwraca `None` dla pustego rejestru; rzuca `KeyError` dla braku wersji; 
  rzuca `ValueError`, gdy nie można odzyskać ani paczki, ani pliku. Launcher 
  wychodzi z błędem i viewer nie uruchamia się w tych przypadkach.
* `drone_physics.run_simulation(..., policy_gains=…)` wywołuje 
  `gains.apply_to(controller)` przed startem pętli symulacji — więc ziarno 
  kaskadowego PID jest wytrenowanym wektorem zamiast domyślnych wartości.

Flaga jest **opcjonalna (opt-in)**. Wywołanie `--physics-live` bez niej 
zachowuje domyślne produkcyjne PID-y, więc istniejące workflow pozostają bez 
zmian.

## Gdzie trafiają moduły po wejściu ciężkiej gałęzi ML

| Moduł                | Dzisiaj (CI)                       | Odroczone (nightly lane)                 |
|:---------------------|:-----------------------------------|:-----------------------------------------|
| `sar_targets.py`     | ✅ dostarczone                      | bez zmian                                |
| `coco_annotator.py`  | projekcja pinhole                  | parametry kamery Gazebo + pozy GT        |
| `image_augment.py`   | Pillow (CPU)                       | Albumentations (CPU/GPU)                 |
| `model_zoo.py`       | mock + 6 stubów                    | YOLO/RT-DETR/DETR/FCOS/ONNX/TensorRT     |
| `inference_logger.py`| JSONL na dysku                     | bez zmian (konsumowane przez Jetson)     |
| `hard_example_miner.py` | pasmo pewności + missed         | bez zmian (output → CVAT/Label Studio)   |
| `model_registry.py`  | płaski JSON                        | opcjonalne lustro W&B                    |
| `kpi.py`             | tylko progi                        | + walidator Ultralytics → wypełnia `kpis`|
| `waypoint_optimizer.py` | random search                   | PPO/SAC, CMA-ES, Bayesian opt, PX4-SITL  |
| `waypoint_kpi.py`    | progi + bramka completion/rmse     | bez zmian (konsumowane przez trenerów)   |

Kontrakt brzmi: **wszystko, co CI dostarcza dzisiaj, jest API widocznym dla 
użytkownika.** Gdy wejdzie ciężki backend, podłączy się on pod istniejące 
powierzchnie (`Detector` ABC, `InferenceLogger`, `ModelRegistry`) bez łamania 
kodu wywołującego.
