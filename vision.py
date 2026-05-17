import pybullet as p
import cv2
import numpy as np

class VisionSystem:
    def __init__(self, camera_pos=[0.3, 1, 0.5], target_pos=[0.0, 0.0, 0.0], width=640, height=480):
        """
        Inicjalizacja kamery.
        camera_pos: Gdzie "wisi" obiektyw (X, Y, Z) - domyślnie na wysokości 0.5m
        target_pos: Gdzie patrzy obiektyw (X, Y, Z) - domyślnie na stół (Z=0)
        """
        self.width = width
        self.height = height
        
        # Zapisujemy parametry do późniejszej transformacji (piksele -> metry)
        self.camera_z = camera_pos[2]
        self.center_x = target_pos[0]
        self.center_y = target_pos[1]

        # 1. Macierz Widoku (skąd i dokąd patrzymy)
        # Używamy lookAt, co jest prostsze niż liczenie kątów (Yaw/Pitch/Roll)
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=[1, 0, 0] # Wskazuje, gdzie jest "góra" obrazu (ważne dla orientacji)
        )
        
        # 2. Macierz Projekcji (parametry soczewki)
        self.fov = 60 # Kąt widzenia w stopniach
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=float(self.width) / self.height,
            nearVal=0.1,
            farVal=2.0
        )

    def get_image(self):
        """Pobiera zdjęcie z symulacji i przerabia na format dla OpenCV."""
        # PyBullet robi zdjęcie
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # rgbImg to płaska tablica z danymi RGBA, zmieniamy ją na macierz (Wysokość x Szerokość x 4)
        img_array = np.array(rgbImg, dtype=np.uint8).reshape((self.height, self.width, 4))
        
        # Odrzucamy 4. kanał (Alpha/Przezroczystość), zostaje RGB
        img_rgb = img_array[:, :, :3]
        
        # Konwertujemy RGB na BGR (OpenCV natywnie używa BGR)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        
        return img_bgr

    def detect_blocks(self, image):
        # 1. Definicja zakresów kolorów w przestrzeni HSV
        # Wartości (Hue, Saturation, Value) dla OpenCV. 
        # Być może będziesz musiał je lekko dostroić do oświetlenia w PyBullet.
        color_ranges = {
            "czerwony": (np.array([0, 100, 100]), np.array([10, 255, 255])),
            "zielony": (np.array([40, 100, 100]), np.array([80, 255, 255])),
            "niebieski": (np.array([100, 100, 100]), np.array([140, 255, 255]))
        }
        
        # Konwersja obrazka do formatu HSV (łatwiej w nim wykrywać kolory niż w BGR)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # TWORZYSZ PUSTĄ LISTĘ NA SAMYM POCZĄTKU
        detected_blocks = [] 

        for color_name, (lower, upper) in color_ranges.items():
            # Tworzenie maski (czarno-biały obraz, gdzie biały to nasz szukany kolor)
            mask = cv2.inRange(hsv_image, lower, upper)
            
            # Wyszukiwanie konturów na masce
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                # Odrzucamy bardzo małe plamki (szum)
                if cv2.contourArea(cnt) > 200:
                    
                    # Obliczanie środka ciężkości konturu (momenty geometryczne)
                    M = cv2.moments(cnt)
                    
                    # Zabezpieczenie przed dzieleniem przez zero
                    if M["m00"] == 0:
                        continue
                        
                    # Wyliczenie współrzędnych cx, cy na obrazie w pikselach
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Przeliczanie pikseli na świat
                    world_x, world_y = self.pixels_to_meters(cx, cy)
                    
                    # PAKUJESZ WSPÓŁRZĘDNE DO SŁOWNIKA I DODAJESZ DO LISTY
                    detected_blocks.append({
                        "kolor": color_name,
                        "piksele": (cx, cy),
                        "wspolrzedne_xyz": [world_x, world_y, 0.05], # Wysokość 0.05 (np. pół wysokości klocka)
                        "kontur": cnt
                    })
                        
        # NAJWAŻNIEJSZE: ZWRACASZ LISTĘ NA ZEWNĄTRZ
        return detected_blocks
    def pixels_to_meters(self, px, py):
        """
        Tłumaczy współrzędne z ekranu (piksele) na współrzędne świata 3D (metry).
        Uwaga: Ta uproszczona matematyka działa dobrze tylko, gdy kamera patrzy idealnie pionowo w dół!
        """
        # Obliczamy, jak bardzo piksel jest oddalony od środka ekranu
        dx_pixels = px - (self.width / 2)
        dy_pixels = py - (self.height / 2)
        
        # Skala: ile metrów przypada na jeden piksel na wysokości stołu
        # Używamy funkcji trygonometrycznej: tan(kąt widzenia / 2) * wysokość kamery
        fov_rad = np.radians(self.fov)
        view_width_meters = 2 * self.camera_z * np.tan(fov_rad / 2)
        meters_per_pixel = view_width_meters / self.width
        
        # Przesunięcie w metrach od środka kamery
        # Zależnie od tego jak zdefiniowaliśmy "górę" kamery, osie X i Y mogą być zamienione
        # Dla cameraUpVector=[1,0,0], X na ekranie odpowiada osi -Y w świecie, a Y osi X.
        dy_meters = -dx_pixels * meters_per_pixel
        dx_meters = -dy_pixels * meters_per_pixel 
        
        # Dodajemy przesunięcie do współrzędnych środka, na które patrzy kamera
        world_x = self.center_x + dx_meters
        world_y = self.center_y + dy_meters
        
        return world_x, world_y

    def draw_detections(self, image, detected_blocks):
        """Rysuje ładne prostokąty i kropki na obrazie do okienka podglądu."""
        img_copy = image.copy()
        
        # Mapowanie koloru ramki w BGR do nazwy
        bgr_colors = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "blue": (255, 0, 0)
        }
        
        for block in detected_blocks:
            kolor_ramki = bgr_colors.get(block["kolor"], (255, 255, 255))
            
            # Kropka na środku
            cx, cy = block["piksele"]
            cv2.circle(img_copy, (cx, cy), 5, kolor_ramki, -1)
            
            # Prostokąt dookoła klocka
            x, y, w, h = cv2.boundingRect(block["kontur"])
            cv2.rectangle(img_copy, (x, y), (x+w, y+h), kolor_ramki, 2)
            
            # Tekst (Kolor i współrzędne 3D)
            world_x, world_y, _ = block["wspolrzedne_xyz"]
            tekst = f"{block['kolor']} (X:{world_x:.2f}, Y:{world_y:.2f})"
            cv2.putText(img_copy, tekst, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, kolor_ramki, 2)
            
        return img_copy