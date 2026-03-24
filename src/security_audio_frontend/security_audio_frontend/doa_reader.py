class DoaReader:
    def __init__(self):
        self.latest_doa_deg = 0.0

    def update_doa(self, doa_deg: float):
        self.latest_doa_deg = float(doa_deg)

    def get_latest_doa(self) -> float:
        return self.latest_doa_deg