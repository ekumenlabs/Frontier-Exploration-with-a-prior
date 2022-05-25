from shapely.geometry import Polygon

class Square(Polygon):
  def __init__(self, side:float, start_x: float=0, start_y: float=0):
    end_x = start_x+side
    end_y = start_y+side
    return super().__init__([(start_x, start_y), (end_x, start_y), (end_x, end_y), (start_x, end_y)])