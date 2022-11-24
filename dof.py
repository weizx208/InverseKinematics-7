class DegreeOfFreedom:
    def __init__(self, id: int, min: float, max: float): 
        self.id = id
        self.min_val: int = min
        self.max_val: int = max

    def control(self, start_val: float, delta: float) -> bool:
        if start_val + delta > max or start_val + delta < min:
            return False
        return True
