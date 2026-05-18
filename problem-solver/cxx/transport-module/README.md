# transport-module

C++ модуль для платформы [OSTIS](https://github.com/ostis-ai), реализующий мультиагентное моделирование транспортировки грузов мобильными наземными роботами.

Разработан в рамках курсового проекта по дисциплине «Математические основы интеллектуальных систем», БГУИР, 2026.

---

## Содержание

- [Описание](#описание)
- [Архитектура](#архитектура)
- [Агенты](#агенты)
- [База знаний](#база-знаний)
- [Структура модуля](#структура-модуля)
- [Запуск тестов](#запуск-тестов)

---

## Описание

Модуль моделирует работу группы из **N** мобильных роботов, перевозящих **M** коробок из пункта **A** в пункт **B** по различным маршрутам. Все агенты взаимодействуют через единую семантическую память (sc-память), реагируя на изменения состояний в SC-графе через механизм событий `ScEvent`.

**Параметры моделирования:**

| Параметр | Описание |
|---|---|
| `N` | Количество роботов |
| `M` | Количество коробок |
| `S` | Расстояние между точками маршрута |
| `V` | Скорость движения роботов |
| `nrel_min/max_load_time` | Диапазон времени загрузки (сек) |
| `nrel_min/max_unload_time` | Диапазон времени разгрузки (сек) |

**Итоговые показатели**, которые собирает система:

- `T_general` — общее время эксперимента
- `T_АБ` — суммарное время загрузки и разгрузки
- `T_преп` — суммарное время ожидания из-за препятствий

---

## Архитектура

Все агенты подписываются на события изменения SC-графа и реагируют только на состояния из своей области ответственности. Взаимодействие между агентами — исключительно через sc-память, без прямых вызовов.

```
sc-память (SC-граф)
    ├── Роботы (concept_mobile_robot)
    ├── Коробки (concept_box)
    ├── Маршрут (concept_route)
    └── Препятствия (concept_obstacle)
         ↑              ↑              ↑              ↑
  InterpretationAgent  CoordinationAgent  AnalyzerAgent  ObstacleAgent
```

---

## Агенты

### `MobileRobotInterpretationAgent`

Агент интерпретации поведения мобильного робота.  
Управляет движением робота по маршруту, обработкой препятствий и обновлением состояний в sc-памяти.

---

#### Реагирует на состояния

| Состояние | Назначение |
|---|---|
| `concept_launched` | инициализация начального состояния робота |
| `concept_box_loaded` | движение к точке разгрузки |
| `concept_box_unloaded` | движение к точке загрузки |
| `concept_stopped` | завершение работы робота |

---

#### Особенности реализации

- Работает через механизм событий `ScEvent`
- Перемещение выполняется по маршруту через `nrel_next_point`
- При обнаружении препятствия робот переходит в состояние ожидания
- Проверка препятствий выполняется циклически с интервалом 100 мс
- Скорость движения хранится в `nrel_robot_current_speed`
- Время перемещения вычисляется как отношение расстояния к скорости
- Положение робота обновляется через `nrel_location`

---

#### Основные методы

| Метод | Назначение |
|---|---|
| `InterpreterStateLaunched()` | запуск робота |
| `InterpreterStateBoxLoaded()` | движение с коробкой |
| `InterpreterStateBoxUnloaded()` | движение без коробки |
| `MoveToNextPoint()` | перемещение в следующую точку |
| `GetNextPoint()` | получение следующей точки маршрута |
| `ObstacleCheck()` | проверка наличия препятствия |
| `StartMoving()` | начало движения |
| `StopMoving()` | остановка движения |
| `SetWaitingState()` | установка состояния ожидания |
| `GetMaxSpeed()` | получение максимальной скорости |
| `GetDistanceToNextPoint()` | получение расстояния до следующей точки |

---

### `MobileRobotCoordinationAgent`

Агент координации процессов загрузки и разгрузки роботов.  
Управляет назначением коробок, синхронизацией доступа к ресурсам и завершением работы роботов.

---

#### Реагирует на состояния

| Состояние | Назначение |
|---|---|
| `concept_ready_being_loaded` | поиск свободной коробки и выполнение загрузки |
| `concept_ready_being_unloaded` | выполнение разгрузки и проверка наличия оставшихся коробок |

---

#### Особенности реализации

- Работает через механизм событий `ScEvent`
- Для синхронизации параллельной работы используются `std::mutex`
- Время загрузки и разгрузки генерируется случайно в заданном диапазоне
- После загрузки коробка связывается с роботом через `nrel_location`
- После разгрузки коробка перемещается в конечную точку маршрута
- При отсутствии свободных коробок робот переводится в состояние `concept_stopped`

---

#### Основные методы

| Метод | Назначение |
|---|---|
| `InterpreterStateReadyBeingLoaded()` | обработка загрузки коробки |
| `InterpreterStateReadyBeingUnloaded()` | обработка разгрузки коробки |
| `AreThereFreeBoxes()` | проверка наличия свободных коробок |
| `GetLoadTime()` | получение времени загрузки |
| `GetUnloadTime()` | получение времени разгрузки |
| `GenerateTime()` | генерация случайного времени операции |
| `ReduceBoxCount()` | уменьшение количества коробок |
| `ChangeActualTempArcToPos/Neg()` | изменение состояний робота |

---

### `MobileRobotAnalyzerAgent`

Агент сбора статистики эксперимента.  
Отслеживает изменения состояний роботов, измеряет продолжительность выполнения операций и вычисляет итоговые показатели моделирования.

---

#### Отслеживаемые состояния

| Состояние | Назначение |
|---|---|
| `concept_launched` | запуск эксперимента и сброс статистики |
| `concept_is_moving` / `concept_is_not_moving` | учёт времени движения |
| `concept_robot_is_waiting` / `concept_robot_is_not_waiting` | учёт времени ожидания препятствий |
| `concept_robot_is_loading` / `concept_robot_is_not_loading` | учёт времени загрузки |
| `concept_robot_is_unloading` / `concept_robot_is_not_unloading` | учёт времени разгрузки |
| `concept_stopped` | вывод статистики по роботу и завершение эксперимента |

---

#### Особенности реализации

- Работает через механизм событий `ScEvent`
- Для каждого робота хранится отдельная статистика (`RobotStats`)
- Время начала состояний сохраняется в `stateStartTimes`
- Идентификация роботов выполняется через `robotAddr.Hash()`
- После завершения всей группы роботов выводится суммарная статистика эксперимента

---

#### Основные показатели

| Показатель | Описание |
|---|---|
| `waitingTime` | время ожидания препятствий |
| `movingTime` | время движения |
| `loadingTime` | время загрузки |
| `unloadingTime` | время разгрузки |
| `totalMovingTime` | суммарное время движения |
| `totalLoadUnloadTime` | суммарное время загрузки и разгрузки |
| `totalWaitingTime` | суммарное время ожидания |
| `experimentStartTime` | время начала эксперимента |

---

#### Основные методы

| Метод | Назначение |
|---|---|
| `CheckInitiationCondition()` | определение обработчика состояния |
| `DoProgram()` | запуск обработки состояния |
| `CalculateDiffInSeconds()` | вычисление длительности состояния |
| `LogTotalStats()` | вывод итоговой статистики |
| `InterpreterState...()` | обработка отдельных состояний робота |

---

#### Пример вывода

```text
====================================
--- robot1 ---
Waiting: 3.008500с
Moving: 6.009118с
Loading: 4.003311с
Unloading: 5.003347с
====================================
---
```

### `RandomObstacleGenerationAgent`

Фоновый генератор случайных препятствий для моделирования динамической среды.  
Работает в отдельном потоке и периодически создаёт препятствия без использования механизма `ScEvent`.

---

#### Параметры генерации

| Константа | Значение | Описание |
|---|---|---|
| `ObstacleProbability` | 1.0 | Вероятность генерации препятствия |
| `MinObstacleInterval` | 1 | Минимальный интервал между генерациями |
| `MaxObstacleInterval` | 1 | Максимальный интервал между генерациями |
| `MinObstacleLifetime` | 8 | Минимальное время жизни препятствия |
| `MaxObstacleLifetime` | 12 | Максимальное время жизни препятствия |

---

#### Принцип работы

Каждый тик (~1 сек) агент:
1. удаляет препятствия с истёкшим временем жизни;
2. проверяет условия генерации;
3. создаёт новое препятствие на случайной точке активного маршрута.

---

#### Особенности реализации

- Использует отдельный поток (`std::thread`)
- Работает через `ScMemoryContext`
- Генерирует препятствия только на маршрутах активных роботов
- Автоматически удаляет препятствия после завершения времени жизни

---

#### Основные методы

| Метод | Назначение |
|---|---|
| `Start()` | Запуск генератора |
| `Stop()` | Остановка генератора |
| `WorkerLoop()` | Основной цикл работы |
| `GenerateStep()` | Выполнение одного шага генерации |
| `GenerateObstacle()` | Создание препятствия |
| `RemoveExpiredObstacles()` | Удаление просроченных препятствий |


---

## База знаний

### Ключевые узлы (`keynodes.hpp`)

**Классы объектов:**
```
concept_mobile_robot   — мобильный робот
concept_box            — коробка
concept_route          — маршрут (структура)
concept_obstacle       — препятствие
concept_distance       — суперкласс для узлов расстояния
```

**Состояния роботов:**
```
concept_launched                  — запущен
concept_stopped                   — остановлен
concept_is_moving                 — движется
concept_is_not_moving             — не движется
concept_robot_is_loading          — загружается
concept_robot_is_not_loading      — не загружается
concept_robot_is_unloading        — разгружается
concept_robot_is_not_unloading    — не разгружается
concept_robot_is_waiting          — ожидает препятствие
concept_robot_is_not_waiting      — не ожидает
concept_ready_being_loaded        — готов к загрузке
concept_ready_being_unloaded      — готов к разгрузке
concept_box_loaded                — коробка на борту
concept_box_unloaded              — коробка снята
```

**Отношения:**
```
nrel_location               — местоположение объекта
nrel_next_point             — следующая точка маршрута
nrel_work_robot_group       — группа роботов эксперимента
nrel_obstacle_position      — положение препятствия
nrel_robot_max_speed        — максимальная скорость робота
nrel_robot_current_speed    — текущая скорость
nrel_min/max_load_time      — диапазон времени загрузки
nrel_min/max_unload_time    — диапазон времени разгрузки
nrel_total_simulation_time      — суммарное время симуляции
nrel_total_load_unload_time     — суммарное время загрузки/разгрузки
nrel_total_obstacle_wait_time   — суммарное время ожидания
rrel_start_point            — начальная точка маршрута (pointA)
rrel_end_point              — конечная точка маршрута (pointB)
```

---

## Структура модуля

```
problem-solver/cxx/transport-module/
├── agents/
│   ├── mobile_robot_analyzer_agent.cpp/.hpp
│   ├── mobile_robot_coordination_agent.cpp/.hpp
│   ├── mobile_robot_interpretation_agent.cpp/.hpp
│   └── random_obstacle_generation_agent.cpp/.hpp
├── keynodes/
│   └── keynodes.hpp                  # все ключевые узлы онтологии
├── test/
│   ├── test-structures/              # SCs-файлы тестовых сценариев
│   │   ├── analyzer_agent_test_1.scs
│   │   ├── analyzer_agent_test_1_initial_states.scs
│   │   ├── analyzer_agent_test_2.scs
│   │   ├── analyzer_agent_test_2_initial_states.scs
│   │   ├── coordination_agent_test_1.scs / _initial_states.scs
│   │   ├── coordination_agent_test_2.scs / _initial_states.scs
│   │   ├── coordination_agent_test_3.scs / _initial_states.scs
│   │   ├── interpretation_agent_test_1.scs / _initial_states.scs
│   │   └── obstacle_generation_agent_test.scs
│   └── units/                        # GTest юнит-тесты
│       ├── test_mobile_robot_analyzer_agent_1.cpp
│       ├── test_mobile_robot_analyzer_agent_2.cpp
│       ├── test_mobile_robot_coordination_agent_1.cpp
│       ├── test_mobile_robot_coordination_agent_2.cpp
│       ├── test_mobile_robot_coordination_agent_3.cpp
│       ├── test_mobile_robot_interpretation_agent_1.cpp
│       └── test_random_obstacle_generation_agent.cpp
├── transport_module.cpp/.hpp         # регистрация агентов
└── CMakeLists.txt
```

---

## Запуск тестов

Сборка с тестами:

```sh
cmake --preset release-with-tests-conan
cmake --build --preset release
```

Запуск всех тестов:

```sh
ctest --preset release
```

Запуск конкретного теста:

```sh
ctest --preset release -R CallMobileRobotAnalyzerAgent1
```

Тесты автоматически обнаруживаются через `cmake_test_discovery`. Каждый тест загружает инициализирующий SCs-файл, подписывает агентов, ждёт завершения и проверяет ожидаемые структуры в SC-графе.



## Технологии

- **Платформа:** [OSTIS / sc-machine](https://github.com/ostis-ai/sc-machine)
- **Язык:** C++17
- **Сборка:** CMake + Conan
- **Тестирование:** GTest (встроен в sc-machine)
- **Формат знаний:** SCs (SC-text Syntax)
