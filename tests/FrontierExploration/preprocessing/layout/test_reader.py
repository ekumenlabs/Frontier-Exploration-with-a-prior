import os

import geopandas as gpd

from FrontierExploration.preprocessing.layout.reader import LayoutReader


class TestLayoutReader:

    def test_clean_and_save_success(self, files_dir):
        small_house_file = files_dir + "small_house.dxf"
        small_clean_house_file = files_dir + "small_clean_house.dxf"

        if os.path.exists(small_clean_house_file):
            os.remove(small_clean_house_file)

        LayoutReader.clean_and_save(small_house_file, small_clean_house_file, ["WALL"], ["LINE"])

        assert os.path.exists(small_clean_house_file)

        xmin, ymin, xmax, ymax = gpd.read_file(small_clean_house_file)['geometry'].total_bounds
        
        assert xmax == 143.2270739724198
        assert xmin == -648.77292602758
        assert ymax == 347.3671317615638
        assert ymin == -216.6328682384362


# class TestOccupancyDataFrame:
