# 3-Osiowy Manipulator Robotyczny

## Opis projektu

Aplikacja implementuje symulator 3-osiowego ramienia robotycznego z:
- **Wizualizacją 3D** (OpenGL)
- **Sterowaniem klawiszami** w czasie rzeczywistym
- **Wyświetlaniem kątów** w terminalu
- **Odwrotną kinematiką** (IK) do pozycjonowania po współrzędnych XY

## Struktura projektu

- `robot.py` - Klasa `RobotArm3DOF` z kinematyką przodu i kinematyką wsteczną
- `main.py` - Aplikacja OpenGL z interfejsem użytkownika
- `shapes.py` - Klasy do rysowania podstawowych kształtów 3D

## Instalacja zależności

```bash
pip install -r requirements.txt
```

## Uruchomienie programu

```bash
python main.py
```

## Sterowanie

### Sterowanie przegubami

- **[1]** - Przegub 1 obrot counter-clockwise (lewo)
- **[2]** - Przegub 1 obrot clockwise (prawo)
- **[3]** - Przegub 2 obrot counter-clockwise
- **[4]** - Przegub 2 obrot clockwise
- **[5]** - Przegub 3 obrot counter-clockwise
- **[6]** - Przegub 3 obrot clockwise

### Pozycjonowanie

- **[T]** - Wejdź w tryb pozycjonowania (podaj docelową pozycję X, Y)
  - Program użyje odwrotnej kinematyki (IK) aby osiągnąć cel
- **[R]** - Reset - powrót do pozycji zerowej

### Interfejs

- **[P]** - Wyświetl pełny stan ramienia w terminalu
- **[H]** - Pokaż/ukryj instrukcję sterowania
- **[ESC]** - Zamknij program

## Współrzędne

- **X** - Oś pozioma (lewo-prawo)
- **Y** - Oś pionowa (góra-dół)
- **Zasięg ramienia** - Maksymalny: ~6.0 jednostek od bazy

## Parametry ramienia

Domyślne długości segmentów:
- L1 (pierwszy segment): 2.5
- L2 (drugi segment): 2.5
- L3 (trzeci segment / końcówka): 1.5

## Przykład użycia w kodzie

```python
from robot import RobotArm3DOF

# Utworzenie ramienia
arm = RobotArm3DOF(L1=2.5, L2=2.5, L3=1.5)

# Obrót przegubu
arm.rotate_joint(1, 1)  # Przegub 1, clockwise
arm.rotate_joint(2, -1) # Przegub 2, counter-clockwise

# Pobranie pozycji
x, y = arm.get_end_effector_position()
print(f"End-effector: ({x:.2f}, {y:.2f})")

# Pozycjonowanie
arm.inverse_kinematics_2d(3.0, 2.0)  # Zmierza do (3, 2)

# Wyświetl stan
arm.print_state()
```

## Wyjaśnienie kinematyki

### Kinematyka prosta (Forward Kinematics)
Oblicza pozycję (x, y) końca ramienia na podstawie kątów przegubów.

### Kinematyka odwrotna (Inverse Kinematics)
Na podstawie docelowej pozycji (x, y), znajduje odpowiednie kąty przegubów.
Implementacja używa metody gradient descent (numerycznej optymalizacji).

## Notatki techniczne

- Ramię pracuje w płaszczyźnie 2D (XY)
- Wszystkie kąty są przechowywane w radianach
- Wyświetlanie odbywa się w stopniach dla czytelności
- End-effector pokazany jest na czerwono w wizualizacji
- Każdy przegub ma ograniczenie kątów: [-180°, 180°]
