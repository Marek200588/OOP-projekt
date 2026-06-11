class TeachAndRepeat:
    """
    Moduł pamięci robota. Odpowiada za tryb "Uczenia i Pracy".
    Zapisuje pozycje (współrzędne + stan chwytaka) i pozwala je odtworzyć.
    """

    def __init__(self):
        # Lista przechowująca nagrane punkty w pliku json śmieszna składnia tam ale to json więc można się było spodziewać
        self.waypoints = []

    def record_waypoint(self, target_xyz, gripper_state):
        """
        Zapisuje obecny stan do pamięci.
        Robimy kopię listy list(target_xyz), żeby nie zapisać referencji (czyli żeby punkt nie zmieniał się później sam z siebie).
        """
        waypoint = {
            'xyz': list(target_xyz),
            'gripper': gripper_state
        }
        self.waypoints.append(waypoint)
        
        numer = len(self.waypoints)
        stan_chwytaka = "ZAMKNIĘTY" if gripper_state else "OTWARTY"
        print(f"[Pamięć] Zapisano punkt #{numer}: XYZ={waypoint['xyz']} | Chwytak: {stan_chwytaka}")

    def get_sequence(self):
        """Zwraca całą nagraną sekwencję ruchów."""
        if not self.waypoints:
            print("[Pamięć] Ostrzeżenie: Próbujesz odtworzyć pustą sekwencję!")
        return self.waypoints

    def clear_memory(self):
        """Czyści pamięć (np. gdy użytkownik chce nagrać nowy ruch od zera)."""
        self.waypoints = []
        print("[Pamięć] Wyczyszczono nagraną trasę.")