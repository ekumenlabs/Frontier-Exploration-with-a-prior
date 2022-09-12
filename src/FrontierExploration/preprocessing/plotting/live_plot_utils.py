from typing import Optional
import dash
from dash import html
from dash import dcc
import plotly.graph_objects as go

from FrontierExploration.preprocessing.grid.visibility_grid import VisibilityGrid
from dash.dependencies import Input, Output
import io
import base64
import matplotlib
# this is actually important since we're plotting to a string, and we don't want a window to be created.
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from threading import Lock



class GlobalState:
    """
    Global state for the live plot.
    """
    def __init__(self, grid: Optional[VisibilityGrid] = None):
        self._lock = Lock()
        self._grid = grid
    
    @property
    def grid(self):
        with self._lock:
            return self._grid
    
    def set_grid(self, grid: VisibilityGrid):
        with self._lock:
            self._grid = grid
    def set_lock(self, lock):
        self._lock = lock

class LivePlotter:
    """
    Class responsible to plot a grid live and serve it to an http server.
    """
    gs = GlobalState()
    def __init__(self, grid: VisibilityGrid, lock: Lock):
        self.gs.set_lock(lock)
        LivePlotter.gs.set_grid(grid)
        self._app = dash.Dash(__name__)
        # describe the webpage layout
        self._app.layout = html.Div(
            html.Div(
                [
                    html.H1("Live visibility grid"),
                    html.Img(id='grid'), # img element
                    dcc.Interval(id="interval-component", interval=2 * 1000, n_intervals=0),  # in milliseconds
                ]
            )
        )

        # Multiple components can update everytime interval gets fired.
        @self._app.callback(Output('grid', 'src'), Input("interval-component", "n_intervals"))
        def update_dynamics_odom_graph(_):
            return self.grid_figure(LivePlotter.gs.grid)


    @staticmethod
    def grid_figure(grid: VisibilityGrid) -> go.Figure:
        """
        Creates a figure for the grid plot.
        :param grid: The grid to plot."""
        grid.plot()
        buf = io.BytesIO()
        plt.savefig(buf, format='jpg')
        plt.close()
        data = base64.b64encode(buf.getbuffer()).decode("utf8") # encode to html elements
        return f"data:image/png;base64,{data}"
    
    def run(self):
        self._app.run_server(debug=False)

    def stop(self):
        self._app = None
