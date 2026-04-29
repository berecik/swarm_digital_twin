# 🎓 Tutorial ML — Tworzenie i Wdrażanie Detekcji SAR na Dronach

Ten tutorial prowadzi przez pełny workflow ML w Swarm Digital Twin: od
wygenerowania syntetycznego zbioru danych aż po uruchomienie detekcji na
locącym dronie w podglądzie na żywo. Wykorzystuje **wyłącznie moduły
dostępne w CI** — ciężkie backendy (YOLOv8, ONNX, Jetson) korzystają z
tego samego API, gdy wejdą przez odroczoną gałąź (patrz
[`nightly_lane.md`](nightly_lane.md)).

Jeśli szukasz uzasadnienia projektowego lub referencji modułów,
najpierw przeczytaj [`ml_pipeline.md`](ml_pipeline.md).

Wersja anglojęzyczna: [`ml_tutorial.md`](ml_tutorial.md).

---

## Co zbudujesz

Po zakończeniu tutoriala będziesz miał:

1. 50-klatkowy syntetyczny zbiór SAR w formacie COCO 2017.
2. Wariant z augmentacją dla domain randomization.
3. Mock-detektor przepuszczony end-to-end przez logger inferencji.
4. Zarejestrowany model z KPI ocenionymi wobec bramki akceptacji.
5. Listę "trudnych przykładów" do re-labelingu przez operatora.
6. Ścieżkę podmiany na prawdziwy backend PyTorch, gdy tylko wejdzie.

Czas wykonania: **~10 sekund** na laptopie. GPU nie jest potrzebne.

---

## Wymagania wstępne

```bash
# Katalog główny projektu
cd swarm_digital_twin

# Środowisko Pythona (utworzone przez repozytorium)
source .venv/bin/activate

# Weryfikacja zależności ML (numpy + Pillow są w venv)
python -c "import numpy, PIL; print('OK')"
```

Jeśli pracujesz wewnątrz testów repo:

```bash
.venv/bin/python -m pytest simulation/test_ml/ -q     # 150 testów
```

---

## Krok 1 — Wybierz cele

Otwórz Pythonowy REPL w katalogu `simulation/`:

```bash
cd simulation
python
```

```python
from ml.sar_targets import find, total_count, coco_categories

print(total_count())                  # 21 — aktualny rozmiar katalogu
person   = find("person")             # SARTarget(category_id=1, ...)
casualty = find("casualty")           # SARTarget(category_id=100, ...)
print(coco_categories()[:3])
```

Katalog jest źródłem prawdy dla *każdej* klasy, którą model potrafi
wykrywać. Aby dodać nowy cel SAR — np. "kamizelka odblaskowa" — dopisz
go do `SAR_TARGETS` w `simulation/ml/sar_targets.py` z wolnym
`category_id` ≥ 200, żeby nie kolidował z ID COCO 2017.

---

## Krok 2 — Wygeneruj syntetyczny zbiór danych

Annotator COCO przekształca krotki (poza drona, pozy celów) w etykiety
ground-truth przez model kamery pinhole. W produkcji te krotki
pochodzą z capture'u w świecie SAR Gazebo; na potrzeby tutoriala
syntetyzujemy je w kodzie.

```python
import numpy as np
from ml.coco_annotator import (
    CameraIntrinsics, FrameCapture, Pose, TargetInstance, annotate, write,
)
from ml.sar_targets import find
from pathlib import Path

# Kamera patrząca w dół, odpowiednik OAK-D Pro w rozdzielczości 640x480.
camera = CameraIntrinsics(fx=400.0, fy=400.0,
                           cx=320.0, cy=240.0,
                           width=640, height=480)

frames = []
rng = np.random.default_rng(42)
for i in range(50):
    # Dron zawisa między 20 m a 60 m AGL na losowym azymucie NE.
    drone_x = float(rng.uniform(-20.0, 20.0))
    drone_y = float(rng.uniform(-20.0, 20.0))
    drone_z = float(rng.uniform(20.0, 60.0))

    # Dwa cele na ziemi poniżej drona.
    targets = [
        TargetInstance(
            target=find("casualty"),
            pose=Pose(position=np.array([drone_x + rng.normal(0, 3),
                                         drone_y + rng.normal(0, 3),
                                         0.0])),
        ),
        TargetInstance(
            target=find("car"),
            pose=Pose(position=np.array([drone_x + rng.normal(0, 8),
                                         drone_y + rng.normal(0, 8),
                                         0.0])),
        ),
    ]
    frames.append(FrameCapture(
        image_id=i,
        file_name=f"frame_{i:04d}.png",
        drone_pose=Pose(position=np.array([drone_x, drone_y, drone_z])),
        targets=targets,
    ))

dataset = annotate(frames, camera)
print(f"obrazy:     {len(dataset['images'])}")
print(f"adnotacje:  {len(dataset['annotations'])}")
print(f"kategorie:  {len(dataset['categories'])}")

write(dataset, Path("/tmp/sar/train.json"))
```

Oczekiwane wyjście (liczby mogą się nieznacznie różnić — większość
celów projektuje się w FOV, ale kilka klatek ginie poza kadrem):

```
obrazy:     50
adnotacje:  96    # ~2 na klatkę minus te, które wypadają poza FOV
kategorie:  21
```

Plik wynikowy w `/tmp/sar/train.json` jest w pełni poprawnym zbiorem
COCO 2017 i może być wczytany przez `pycocotools`, trener YOLO
Ultralytics, detectron2 itd. bez żadnych modyfikacji.

---

## Krok 3 — Dodaj domain randomization

Prawdziwe zdjęcia SAR różnią się ogromnie pod względem oświetlenia,
pogody, motion blur i przesłonięcia. Moduł augmentacji symuluje to,
żeby model nie przeuczył się na idealne rendery z Gazebo.

```python
from PIL import Image
from ml.image_augment import Augmenter, AugmentationSpec

# Renderujemy syntetyczną klatkę (w produkcji pochodzi z Gazebo).
img = Image.new("RGB", (640, 480), (135, 145, 155))

aug = Augmenter(spec=AugmentationSpec(), seed=42)

bright_img   = aug.random_brightness(img)
contrast_img = aug.random_contrast(img)
blurred      = aug.random_blur(img)
cutout       = aug.random_cutout(img)
rainy        = aug.add_weather(img, kind="rain")
snowy        = aug.add_weather(img, kind="snow")

# Albo łańcuch wszystkich kroków z prawdopodobieństwem per-krok:
chained = aug.apply_random_chain(img, p=0.7)
chained.save("/tmp/sar/aug_demo.png")
```

**Powtarzalność:** Augmenter seeduje własny `numpy.random.Generator`,
więc dwie instancje `Augmenter(seed=42)` produkują identyczne
bajt-po-bajcie wyjścia. To właśnie na tej własności opiera się suite
regresyjny — nigdy nie polegaj na globalnym stanie `numpy.random`
wewnątrz augmentacji.

**Zastosowanie po stronie zbioru:** Typowy skrypt treningowy owija
loader COCO tak, żeby augmentacja działała raz na obraz na epokę:

```python
import json
from PIL import Image
from pathlib import Path

ds = json.loads(Path("/tmp/sar/train.json").read_text())
aug = Augmenter(seed=0)

for img_meta in ds["images"][:5]:
    # W produkcji każdy obraz leży obok JSON-a COCO.
    img = Image.new("RGB", (img_meta["width"], img_meta["height"]),
                    (135, 145, 155))
    aug.apply_random_chain(img, p=0.5).save(
        f"/tmp/sar/aug_{img_meta['id']:04d}.png")
```

Bounding boxy **nie** są transformowane przez aktualny zestaw
augmentacji (żadna z nich nie rusza pikseli geometrycznie — jasność,
kontrast, blur, cutout, pogoda zachowują geometrię). Gdy wjedzie
odroczona gałąź Albumentations, transformacje geometryczne (rotacja,
crop, flip) pojawią się sparowane z aktualizacjami świadomymi bboxów.

---

## Krok 4 — Wybierz backend modelu

`ModelZoo` to jedyny punkt wejścia. Dziś uruchamialny jest tylko
`mock`; sześć pozostałych backendów jest zarejestrowanych jako
odroczone stuby rzucające `NotImplementedError` z jasnym wskaźnikiem.

```python
from ml.model_zoo import ModelZoo, MockDetector

zoo = ModelZoo()
print(zoo.list())
# ['detr', 'fcos', 'mock', 'onnx', 'rtdetr', 'tensorrt', 'yolo']

detector = zoo.load("mock")          # działa dziś
print(detector.name())               # "mock"
```

Próba użycia odroczonego backendu jest świadomie jawna:

```python
try:
    zoo.load("yolo", weights_path="yolov8n.pt")
except NotImplementedError as e:
    print(e)
# yolo backend requires the optional `ultralytics` dependency
# and is part of the deferred ML branch — see `docs/nightly_lane.md` ...
```

**Własny backend.** Gdy będziesz gotów dostarczyć prawdziwy model
PyTorch, zarejestruj go raz przy starcie:

```python
from ml.model_zoo import register, Detector, Detection

class YoloV8Backend(Detector):
    backend = "yolo"
    def __init__(self, weights_path: str, **kw):
        from ultralytics import YOLO
        self._model = YOLO(weights_path)
    def detect(self, image):
        results = self._model(image, verbose=False)[0]
        return [
            Detection(category_id=int(box.cls),
                      category_name=results.names[int(box.cls)],
                      bbox=tuple(box.xywh[0].tolist()),
                      confidence=float(box.conf))
            for box in results.boxes
        ]

# Rejestrujemy nadpisując stub:
register("yolo", lambda weights_path=None, **kw:
         YoloV8Backend(weights_path=weights_path, **kw))
```

Kod wywołujący pozostaje nietknięty: `zoo.load("yolo", weights_path="...")`.

---

## Krok 5 — Loguj inferencje do JSONL

Logger inferencji zapisuje każdą detekcję z metadanymi klatki, dzięki
czemu można odtwarzać i analizować zachowanie modelu bez jego
ponownego uruchamiania.

```python
from pathlib import Path
import numpy as np
from ml.inference_logger import InferenceLogger, read_log

log_path = Path("/tmp/sar/run_001.jsonl")
detector = ModelZoo().load("mock")

with InferenceLogger(log_path, model_version="mock_v1") as log:
    for i in range(10):
        # W produkcji: img = camera.capture()
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        detections = detector.detect(img)
        log.log(detections,
                image_path=f"frame_{i:04d}.png",
                metadata={"weather": "clear", "expected_targets": 2})

print(f"zalogowano {sum(1 for _ in read_log(log_path))} rekordów")
```

Podgląd pliku:

```bash
head -1 /tmp/sar/run_001.jsonl | python -m json.tool
```

```json
{
  "frame_id": 0,
  "t_wall_s": 1745251200.123,
  "model_version": "mock_v1",
  "image_path": "frame_0000.png",
  "metadata": {"weather": "clear", "expected_targets": 2},
  "detections": [
    {"category_id": 1, "category_name": "person",
     "bbox": [50.0, 50.0, 30.0, 60.0], "confidence": 0.92},
    {"category_id": 100, "category_name": "casualty",
     "bbox": [200.0, 150.0, 90.0, 35.0], "confidence": 0.74}
  ]
}
```

**Odzyskiwanie po awarii.** Otwórz tę samą ścieżkę nowym loggerem, a
numeracja klatek wznowi się od ostatniego poprawnie zapisanego
rekordu. Jeśli poprzedni run padł w trakcie pisania linii, niepełny
rekord zostanie po cichu pominięty.

```python
with InferenceLogger(log_path, model_version="mock_v1") as log:
    log.log([])     # ten otrzymuje frame_id = 10 (kontynuacja)
```

---

## Krok 6 — Wyłów trudne przykłady

Skoro masz już log inferencji, wyciągnij klatki wymagające uwagi
operatora. Dzisiaj shippujemy dwie heurystyki:

```python
from ml.hard_example_miner import mine, MinerConfig
from ml.inference_logger import read_log

records = list(read_log(log_path))
hard = mine(records)
for ex in hard[:5]:
    print(ex.reason, ex.score, ex.image_path)
```

**`uncertain`** wyzwala się, gdy co najmniej jedna detekcja ma
confidence w pasmie `[0.30, 0.60]` — model nie jest pewny. Wynik
osiąga szczyt 1.0 gdy detekcja jest w środku pasma (0.45) i maleje
ku jego krawędziom.

**`missed`** wyzwala się, gdy model zwrócił zero detekcji, a
`metadata["expected_targets"]` podane przez operatora wynosi ≥ 1.
Wynik równa się liczbie oczekiwanych celów, więc klatki, w których
model przeoczył wiele ofiar, wypływają na szczyt.

Dostrój progi do swojego zbioru:

```python
cfg = MinerConfig(low_conf=0.20, high_conf=0.70)   # poszerz pasmo
hard = mine(records, cfg)
```

Wyjście jest kolejką wejściową dla narzędzia do labelingu (CVAT, Label
Studio, itp.). Push do CVAT dzieje się w odroczonej gałęzi — na razie
zapisz JSON-owy sidecar:

```python
import json
Path("/tmp/sar/relabel_queue.json").write_text(json.dumps(
    [ex.to_dict() for ex in hard], indent=2))
```

---

## Krok 7 — Zarejestruj i promuj model

Po treningu zarejestruj model, żeby przyszłe runy mogły go odnaleźć i
porównywać się z nim.

```python
from ml.model_registry import ModelEntry, ModelRegistry

reg = ModelRegistry(Path("/tmp/sar/registry.json"))

reg.register(ModelEntry(
    version="mock_v1",
    backend="mock",
    weights_path="(brak — mock)",
    parent_version=None,
    kpis={"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
    notes="Model bazowy tutoriala.",
))

print(reg.latest().version)         # 'mock_v1'
print(reg.best_by_kpi("mAP_50"))    # ModelEntry(version='mock_v1', ...)
```

Kiedy wytrenujesz nowego kandydata, oceń go przez bramkę zanim
zarejestrujesz:

```python
from ml.kpi import evaluate_kpis, compare_models

candidate = {"mAP_50": 0.83, "recall": 0.88, "inference_ms": 40.0}
verdict = evaluate_kpis(candidate)
print(verdict.verdict, verdict.failures)    # 'PASS', []

incumbent = reg.best_by_kpi("mAP_50").kpis
promote, reason = compare_models(candidate, incumbent)
print(promote, reason)
# True 'primary KPI improved by +0.0500'

if promote:
    reg.register(ModelEntry(
        version="mock_v2", backend="mock", weights_path="(mock)",
        parent_version="mock_v1", kpis=candidate,
        notes="Szerszy łańcuch augmentacji; +5 pp mAP.",
    ))
```

**Chodzenie po linii przodków** pozwala prześledzić dowolny model aż
do korzenia:

```python
for entry in reg.lineage("mock_v2"):
    print(entry.version, "←", entry.parent_version)
# mock_v2 ← mock_v1
# mock_v1 ← None
```

Ręczna edycja pliku JSON wprowadzająca cykl (np. v1 ← v2 ← v1) jest
wykrywana i rzuca wyjątek zamiast się zapętlać.

---

## Krok 8 — Ściągawka bramki promocji

Bramka akceptacji to **bezwzględna podłoga**:

| Metryka        | Próg      | Tryb błędu                             |
|:---------------|:---------:|:---------------------------------------|
| `mAP_50`       | ≥ 0.75    | "mAP_50 0.62 < target 0.75"            |
| `recall`       | ≥ 0.85    | "recall 0.40 < target 0.85"            |
| `inference_ms` | ≤ 50.0    | "inference_ms 120.0 > budget 50.0"     |

Bramka promocji (`compare_models`) dodaje dwie **względne** reguły na
wierzchu bramki akceptacji:

1. Opóźnienie nie może się pogorszyć o więcej niż 0.5 ms.
2. Główny KPI (`mAP_50`) musi się poprawić o ≥ 0.005.

Recall **nie** jest bramkowany po delcie — tylko po bezwzględnej
podłodze — więc kandydat z `mAP +0.01 / recall −0.04` zostanie
promowany, o ile recall ≥ 0.85. Zgadza się to z "główny KPI = mAP",
ale warto o tym wiedzieć podczas triażu porażek promocji.

---

## Krok 9 — Podłącz do drona

Gdy model zostanie promowany, runtime wiring jest identyczny dla
mocka i prawdziwych backendów:

```python
from ml.model_zoo import ModelZoo
from ml.inference_logger import InferenceLogger
from ml.model_registry import ModelRegistry
from pathlib import Path

reg = ModelRegistry(Path("/var/lib/swarm/models/registry.json"))
active = reg.best_by_kpi("mAP_50")

zoo = ModelZoo()
detector = zoo.load(active.backend, weights_path=active.weights_path)

with InferenceLogger(
    Path(f"/var/lib/swarm/logs/{active.version}.jsonl"),
    model_version=active.version,
) as log:
    while flight_active():                        # Twoja pętla misji
        frame, metadata = camera.capture()
        detections = detector.detect(frame)
        log.log(detections, image_path=metadata["filename"],
                metadata={"weather": metadata.get("weather", "unknown"),
                          "expected_targets": metadata.get("expected", 0)})
        publish_to_zenoh(detections)              # konsumenci downstream
```

Runtime Jetson, który wchodzi z odroczoną gałęzią, użyje dokładnie tej
ścieżki kodu. Jedyną zmianą jest `zoo.load("yolo", ...)` zamiast
`zoo.load("mock")`, gdy tylko prawdziwy backend zostanie
zarejestrowany.

---

## Krok 10 — Uruchom testy

Projekt ma **150 testów ML w 12 plikach** (8 detekcyjnych + 1
integracyjny + 2 sterowania). Uruchamiaj je przy każdej zmianie w
`simulation/ml/`:

```bash
# Tylko pipeline ML
.venv/bin/python -m pytest simulation/test_ml/ -q

# Tylko przepływy integracyjne (cross-modułowe)
.venv/bin/python -m pytest simulation/test_ml/test_pipeline_integration.py -v

# Tylko optymalizator waypointów (strona sterowania)
.venv/bin/python -m pytest simulation/test_ml/test_waypoint_optimizer.py simulation/test_ml/test_waypoint_kpi.py -q

# Pełny suite projektu (603 w tym ML)
.venv/bin/python -m pytest simulation/ -q
```

Testy są zorganizowane tak, że dodanie nowego modułu do
`simulation/ml/` powinno iść w parze z nowym
`simulation/test_ml/test_<nazwa>.py` oraz odpowiednimi edycjami w
`test_pipeline_integration.py`.

---

## Typowe pułapki

| Objaw | Przyczyna | Rozwiązanie |
|:--|:--|:--|
| `KeyError: 'unknown SAR target'` | Źle napisana nazwa klasy | `find()` rzuca z listą dostępnych — skopiuj poprawną z komunikatu. |
| Wszystkie projekcje zwracają `None` | `dz <= 0` (kamera poniżej celu) | Sprawdź, czy `drone_pose.position[2]` jest większe niż `position[2]` każdego celu. |
| Wyjścia augmentera różnią się przy powtórce | Użyto globalnego `numpy.random` zamiast seedowanego generatora | Używaj zawsze `Augmenter(seed=...)` i nie dotykaj `np.random.seed()` w augmentacjach. |
| `NotImplementedError: yolo backend requires ...` | Próba załadowania odroczonego backendu | Zainstaluj brakującą zależność i `register()` własny loader, albo używaj `mock` podczas developmentu. |
| `ValueError: cycle detected in lineage` | Ręcznie zedytowany registry.json z cyklem `parent_version` | Przejrzyj i popraw JSON. Kontrola jest celowa — nigdy jej nie wyłączaj. |
| `ValueError: candidate fails acceptance gate` | KPI poniżej bezwzględnej podłogi | Zajrzyj w `evaluate_kpis(candidate).failures` po powód per-metryka. |

---

## Bonus — Optymalizacja osiągania waypointów (pojedynczy dron)

Pipeline powyżej optymalizuje *to, co dron widzi*. Strona sterowania
(`simulation/ml/waypoint_optimizer.py` + `waypoint_kpi.py`) optymalizuje
*jak dobrze do nich dolatuje*. Ten sam rejestr, ten sam kształt bramki KPI.

### Uruchom z powłoki (3 polecenia)

```bash
# 1. Wytrenuj nastrojony PID na misji patrol (5 prób, ~10 s).
./scripts/ml_train_waypoint.sh --trials 5 --mission patrol

# 2. Trudniejszy wariant na patrol + lawnmower (16 prób, ~3 min).
./scripts/ml_train_waypoint.sh --trials 16 --missions patrol,lawnmower

# 3. Oceń ostatnio zarejestrowaną politykę na każdej misji.
./scripts/ml_evaluate_waypoint.sh \
    --missions patrol,lawnmower,escort,heavy_lift

# 4. Lataj wytrenowaną polityką w live viewerze pojedynczego drona.
./run_scenario.sh --physics-live --policy=pid_v1
```

Artefakty trafiają do `reports/ml_waypoint/<znacznik_czasu>/`:

```
policy_registry.json    # ModelRegistry — jeden wpis na promowaną politykę
pid_v1_gains.json       # 18-elementowy wektor wzmocnień
trial_history.json      # każda próba: (gains, metrics, objective)
summary.json            # czytelne podsumowanie
```

### Albo z poziomu Pythona

```python
from ml.waypoint_optimizer import (
    PolicyGains, evaluate_policy, run_episode, random_search,
)
from ml.waypoint_kpi import compare_waypoint_policies, evaluate_waypoint_kpis

# Uruchom domyślny kontroler produkcyjny przez misję patrol.
baseline = PolicyGains.from_baseline()
metrics = run_episode(baseline, mission_kind="patrol", max_time=60.0)
print(f"completion={metrics.completion_ratio:.2f} "
      f"rmse_settled={metrics.rmse_xyz_m:.3f} m")

# Strojenie przez random search — pierwsza próba to zawsze baseline.
result = random_search(n_trials=8, seed=0,
                        mission_kinds_to_run=["patrol"],
                        max_time=60.0)
print(f"najlepszy objective: {result.best_objective:.3f}")
print(f"najlepsze gains: {result.best_gains}")

# Bramka promocji wobec produkcyjnego baseline'u.
baseline_metrics = evaluate_policy(baseline, max_time=60.0)
promote, reason = compare_waypoint_policies(
    result.best_metrics, baseline_metrics)
print(promote, reason)
```

### Jakie metryki mają znaczenie i dlaczego

| KPI | Znaczenie | Próg |
|:--|:--|:--:|
| `completion_ratio` | Frakcja osiągniętych waypointów. | ≥ 0.95 |
| `rmse_xyz_m` | Błąd ustabilizowanego śledzenia — *tylko* próbki, gdy dron jest wewnątrz capture radius. | ≤ 1.0 m |
| `time_to_first_wp_s` | Czas do osiągnięcia + utrzymania pierwszego waypointa. | ≤ 30 s |
| `max_overshoot_m` | Najgorsze przekroczenie poza waypoint w strefie capture (wyłapuje leniwe hamowanie bez liczenia dystansu tranzytu). | ≤ 6.0 m |
| `energy_proxy_j` | `∫ thrust dt` — surogat zużycia baterii. Śledzony, nie progowany — używany jako bramka braku regresji przy promocji. | — |

**Logika promocji** (`compare_waypoint_policies`):

1. Kandydat musi przejść bezwzględną podłogę (`evaluate_waypoint_kpis`).
2. Energia nie może się pogorszyć.
3. `completion_ratio` musi się poprawić o ≥ 0.01 *lub* — przy remisie
   completion — RMSE musi się zacieśnić o ≥ 0.05 m.

Oznacza to, że polityka która trafia w te same waypointy, ale trzyma
je ciaśniej, też zasługuje na promocję.

### Kiedy retrenować per misja

Domyślna bramka akceptacji jest skalibrowana dla `patrol` i
`lawnmower`. Pozostałe dwie misje (`escort`, `heavy_lift`) prowadzą
drona w ciaśniejszych okręgach wokół ruchomych formacji. Jeśli
nastroisz *tylko* na `patrol` i potem oceniasz na `lawnmower`,
często zobaczysz przekroczenie progu overshoot — kontroler nie był
wystawiony na dynamikę 40 m tranzytu. Rozwiązanie: przekaż misje,
które faktycznie polecisz, do `--missions` na etapie treningu, np.:

```bash
./scripts/ml_train_waypoint.sh \
    --trials 32 --missions patrol,lawnmower,escort
```

Liczba prób skaluje się liniowo: 32 próby × 3 misje × ~3 s/epizod ≈ 5 min.

### Gdzie wpina się odroczona gałąź

`PolicyGains` to interfejs trenowanych "wag". Prawdziwe trenery RL
(PPO, SAC), CMA-ES, optymalizacja Bayesowska oraz strojenie w pętli
PX4-SITL wszystkie wpinają się w ten sam kształt `PolicyGains` — patrz
`docs/nightly_lane.md` dla kontraktu. Dzisiejszy `random_search` to
CI-deliverable baseline, który dowodzi, że powierzchnia działa
end-to-end.

### Latanie wytrenowaną polityką

Pojedynczo-dronowa ścieżka fizyki Pythona
(`./run_scenario.sh --physics-live`) akceptuje tę samą wytrenowaną
politykę jako flagę CLI — bez zmian w kodzie:

```bash
# Auto-wybór najnowszej polityki (najlepsza po completion).
./run_scenario.sh --physics-live --policy=pid_v1

# Wskaż jawnie ścieżkę rejestru, gdy istnieje wiele raportów.
./run_scenario.sh --physics-live \
    --policy=pid_v2 \
    --policy-registry=/sciezka/policy_registry.json

# Pętla demo + użycie wytrenowanej polityki.
./run_scenario.sh --physics-live --loop --policy=pid_v1
```

Flaga jest przepuszczana przez
`physics_live_replay.load_policy_gains()` do
`drone_physics.run_simulation(policy_gains=…)`, co seeduje kaskadowy
`PositionController` przed wystartowaniem pętli. Wywołanie bez
`--policy` zachowuje produkcyjne PID-y — flaga jest opt-in. Linia
startowa potwierdza, która polityka się załadowała:

```
Using trained PID policy pid_v1 from reports/ml_waypoint/<ts>/policy_registry.json
```

Jeśli rejestru nie da się znaleźć lub wersja nie istnieje, launcher
kończy się jasnym błędem przed startem viewera — typo w nazwie
polityki kończy się szybko, zamiast cicho lecieć baseline'em.

---

## Co dalej

Gdy przejdziesz przez powyższe kroki, masz kompletny model mentalny
pipeline'u ML tak, jak shippuje w CI. Odroczona gałąź
(`docs/nightly_lane.md`) wpina się w te same powierzchnie:

- Podmiana `mock` w `model_zoo` na YOLO/RT-DETR/ONNX/TensorRT.
- Podmiana syntetycznych list `FrameCapture` na capture ze świata SAR w Gazebo.
- Podmiana augmentacji Pillow na Albumentations (CPU/GPU).
- Push kolejki trudnych przykładów do CVAT lub Label Studio.
- Mirror rejestru do Weights & Biases dla widoczności cross-team.

Każda z tych zmian to **podmiana pod istniejącym API** — żadna nie
wymaga modyfikowania kodu wywołującego, który już korzysta z modułów
udokumentowanych tutaj.

Pełna referencja per-moduł: [`ml_pipeline.md`](ml_pipeline.md).
