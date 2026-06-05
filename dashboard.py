#!/usr/bin/env python3
"""
Mars Rover MCT  —  Plotly Dash dashboard
Usage:  pip install dash plotly
        python3 dashboard.py
Open:   http://localhost:8050

Launches ./mars_rover as a subprocess.  A background thread reads its
stdout and parses telemetry; Dash callbacks poll shared state every 500 ms.
Commands are written to the process's stdin (handled by the MCT FreeRTOS task).
"""

import collections, os, re, subprocess, sys, threading, time
from datetime import datetime

import dash
from dash import Input, Output, ctx, dash_table, dcc, html
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ═══════════════════════════════════════════════════ palette ═══════════════
BG    = '#0d1117'
CARD  = '#161b22'
BDR   = '#30363d'
BLUE  = '#58a6ff'
GREEN = '#3fb950'
AMBER = '#d29922'
RED   = '#f85149'
DIM   = '#8b949e'
FG    = '#c9d1d9'
FONT  = '"Courier New", Courier, monospace'

PLOT_LAYOUT = dict(
    plot_bgcolor=CARD, paper_bgcolor=CARD,
    font=dict(family=FONT, color=DIM, size=10),
    margin=dict(l=44, r=16, t=8, b=36),
    legend=dict(font=dict(size=10), bgcolor='rgba(0,0,0,0)', x=0, y=1),
)
AXIS = dict(color=DIM, gridcolor='#21262d', showgrid=True,
            zerolinecolor=BDR, tickfont=dict(size=9))

# ═══════════════════════════════════════════════════ shared state ══════════
MAX_HIST = 120
MAX_WP   = 50
MAX_LOG  = 200
MAX_CMDS = 20

_lock = threading.Lock()
_t0   = time.time()

# scalars (updated by reader thread)
_x = _y = _hdg = _path = _obs = _power = _alt = _lidar = 0
_orig = _temp = 0.0
_wp_count = 0

# time-series
_ts: dict = {k: collections.deque(maxlen=MAX_HIST)
             for k in ('t_s', 'temp', 'lidar', 'alt', 't_p', 'power')}

# structured lists / deques
_waypoints: list = []
_earth_cmds: collections.deque = collections.deque(maxlen=MAX_CMDS)
_status_log: collections.deque = collections.deque(maxlen=MAX_LOG)
_mct_sent:   collections.deque = collections.deque(maxlen=MAX_CMDS)

# last-seen timestamp (elapsed s) per task, updated by reader thread
_last_seen: dict = {k: None for k in [
    'MainComputer', 'Motor', 'Comms', 'SelfMonitor',
    'Sensor', 'Navigation', 'Power', 'MCT',
]}

_events: dict = {
    'OBSTACLE': False, 'LOW_POWER': False, 'COMMS_LOST': False,
    'SENSOR_FAULT': False, 'MOTOR_FAULT': False, 'EMERG_STOP': False,
}

# task health: name → [priority, status]  (list so it's mutable)
_tasks: dict = {
    'MainComputer': [7, 'ALIVE'],
    'Motor':        [6, 'ALIVE'],
    'Comms':        [5, 'ALIVE'],
    'SelfMonitor':  [4, 'ALIVE'],
    'Sensor':       [3, 'ALIVE'],
    'Navigation':   [2, 'ALIVE'],
    'Power':        [3, 'ALIVE'],
}

# ═══════════════════════════════════════════════════ parsers ══════════════
_HDG = {'N': 0, 'E': 90, 'S': 180, 'W': 270}
_LVL = {0: 'OK', 1: 'WARNING', 2: 'ERROR', 3: 'FAULT'}

# [Motor] Pos:(10, 20)cm  Heading:N  Path:150cm  Obstacle:200cm
_RE_POS  = re.compile(r'\[Motor\] Pos:\((-?\d+),\s*(-?\d+)\)cm\s+Heading:(\S+)\s+Path:(\d+)cm\s+Obstacle:(-?\d+)cm')
# [Motor] >> Distance from start: 22.4 cm
_RE_ORIG = re.compile(r'Distance from start:\s*([\d.]+)\s*cm')
# [Sensor] Temp:-63.00 C  Dist:150cm  Alt:0cm
_RE_SENS = re.compile(r'\[Sensor\] Temp:([-\d.]+)\s*C\s+Dist:(\d+)cm\s+Alt:(-?\d+)cm')
# [Power] Estimated usage: 73 / 100 units
_RE_PWR  = re.compile(r'\[Power\] Estimated usage:\s*(\d+)')
# [Navigation] WP#31  Pos:(X,Y)cm  Origin:D.Dcm  Sensor dist:Dcm  Temp:-67.30C
_RE_WP   = re.compile(r'\[Navigation\] WP#(\d+)\s+Pos:\((-?\d+),(-?\d+)\)cm\s+Origin:([\d.]+)cm\s+Sensor dist:(\d+)cm\s+Temp:([-\d.]+)C')
# [Comms] TX >> [Task] Level:N Code:N
_RE_TX   = re.compile(r'\[Comms\] TX >> \[(\w+)\] Level:(\d+) Code:\d+')
# [Comms] RX << Earth cmd: FORWARD  speed:65%
_RE_RX   = re.compile(r'\[Comms\] RX << Earth cmd:\s*(\S+)\s+speed:(\d+)%')
# [SelfMonitor] WARNING: Motor task appears frozen!
_RE_FRZ  = re.compile(r'\[SelfMonitor\] WARNING: (\w+).*frozen')
# [SelfMonitor] Motor task restarted
_RE_RST  = re.compile(r'\[SelfMonitor\] (\w+) (?:task )?restarted')
# task prefix — used for last-seen tracking
_RE_PFX  = re.compile(r'^\[(\w+)\]')


def _parse(line: str) -> None:
    global _x, _y, _hdg, _path, _obs, _orig
    global _temp, _lidar, _alt, _power, _wp_count

    now = datetime.now().strftime('%H:%M:%S')
    elapsed = time.time() - _t0

    with _lock:
        # position
        m = _RE_POS.search(line)
        if m:
            _x = int(m.group(1)); _y = int(m.group(2))
            _hdg  = _HDG.get(m.group(3), 0)
            _path = int(m.group(4)); _obs = int(m.group(5))
            if _obs >= 50:
                _events['OBSTACLE'] = False

        m = _RE_ORIG.search(line)
        if m:
            _orig = float(m.group(1))

        # sensor
        m = _RE_SENS.search(line)
        if m:
            _temp  = float(m.group(1))
            _lidar = int(m.group(2))
            _alt   = int(m.group(3))
            _ts['t_s'].append(elapsed)
            _ts['temp'].append(_temp)
            _ts['lidar'].append(_lidar)
            _ts['alt'].append(_alt)

        # power
        m = _RE_PWR.search(line)
        if m:
            _power = int(m.group(1))
            _ts['t_p'].append(elapsed)
            _ts['power'].append(_power)

        # waypoint
        m = _RE_WP.search(line)
        if m:
            _wp_count = int(m.group(1))
            wp = {'n': _wp_count, 'x': int(m.group(2)), 'y': int(m.group(3)),
                  'orig': float(m.group(4)), 'dist': int(m.group(5)),
                  'temp': float(m.group(6))}
            if len(_waypoints) >= MAX_WP:
                _waypoints.pop(0)
            _waypoints.append(wp)

        # status log (from Comms TX drain)
        m = _RE_TX.search(line)
        if m:
            task_name = m.group(1)
            lvl_int   = int(m.group(2))
            tail = line[m.end():].lstrip()
            msg  = tail.lstrip('—').lstrip('—').strip()
            lvl_str = _LVL.get(lvl_int, 'OK')
            _status_log.append({'time': now, 'task': task_name,
                                 'level': lvl_str, 'message': msg})
            if task_name in _tasks:
                pri = _tasks[task_name][0]
                if lvl_int >= 2:   _tasks[task_name] = [pri, 'FAULT']
                elif lvl_int == 1: _tasks[task_name] = [pri, 'WARN']
                else:              _tasks[task_name] = [pri, 'ALIVE']

        # earth commands
        m = _RE_RX.search(line)
        if m:
            _earth_cmds.append({'time': now, 'cmd': m.group(1),
                                 'speed': m.group(2)})

        # events
        if '[Motor] Obstacle at' in line:           _events['OBSTACLE']     = True
        if '[MainComputer] Obstacle detected' in line: pass   # keep set
        if 'signal LOST'     in line:               _events['COMMS_LOST']   = True
        if 'signal RESTORED' in line:               _events['COMMS_LOST']   = False
        if 'Navigation suspended' in line:          _events['LOW_POWER']    = True
        if 'Navigation resumed'   in line:          _events['LOW_POWER']    = False
        if 'Sensor heartbeat lost'  in line:        _events['SENSOR_FAULT'] = True
        if 'Motor heartbeat lost'   in line:        _events['MOTOR_FAULT']  = True
        if 'EMERGENCY STOP' in line:                _events['EMERG_STOP']   = True

        # task health from SelfMonitor
        m = _RE_FRZ.search(line)
        if m:
            n = m.group(1)
            for k in _tasks:
                if n.lower() in k.lower():
                    _tasks[k][1] = 'FAULT'

        m = _RE_RST.search(line)
        if m:
            n = m.group(1)
            for k in _tasks:
                if n.lower() in k.lower():
                    _tasks[k][1] = 'ALIVE'
                    # clear related fault events
                    if 'sensor' in k.lower():  _events['SENSOR_FAULT'] = False
                    if 'motor'  in k.lower():  _events['MOTOR_FAULT']  = False

        # last-seen: record elapsed time for the task whose prefix appears first
        m_pfx = _RE_PFX.match(line)
        if m_pfx and m_pfx.group(1) in _last_seen:
            _last_seen[m_pfx.group(1)] = elapsed


# ═══════════════════════════════════════════════════ process ══════════════
_proc = None


def _start() -> None:
    global _proc
    binary = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mars_rover')
    if not os.path.exists(binary):
        print(f"Error: '{binary}' not found — run 'make' first.")
        sys.exit(1)
    _proc = subprocess.Popen(
        [binary],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1,
    )
    threading.Thread(target=_reader, daemon=True).start()


def _reader() -> None:
    for line in _proc.stdout:
        _parse(line)


def _send(cmd: str) -> None:
    if _proc and _proc.poll() is None:
        try:
            _proc.stdin.write(cmd + '\n')
            _proc.stdin.flush()
            now = datetime.now().strftime('%H:%M:%S')
            with _lock:
                _mct_sent.append({'time': now, 'cmd': cmd.upper()})
        except OSError:
            pass


# ═══════════════════════════════════════════════════ app ══════════════════
app = dash.Dash(__name__, update_title=None, suppress_callback_exceptions=True)
app.title = 'Mars Rover MCT'

# ── style helpers ─────────────────────────────────────────────────────────
_CARD_S = {
    'background': CARD, 'border': f'1px solid {BDR}',
    'borderRadius': '5px', 'padding': '10px 14px', 'marginBottom': '8px',
}
_LBL_S = {
    'fontSize': '10px', 'color': DIM, 'letterSpacing': '1.2px',
    'textTransform': 'uppercase', 'marginBottom': '4px',
}
_VAL_S = {
    'fontSize': '22px', 'fontWeight': 'bold',
    'fontFamily': FONT, 'lineHeight': '1.1',
}
_BTN_S = {
    'background': CARD, 'color': BLUE, 'border': f'1px solid {BDR}',
    'padding': '8px 18px', 'margin': '3px', 'cursor': 'pointer',
    'fontFamily': FONT, 'fontSize': '13px', 'fontWeight': 'bold',
    'borderRadius': '5px',
}
_ESTOP_S = {**_BTN_S, 'color': RED, 'borderColor': RED}


def _badge(label, active, color=RED):
    return html.Span(label, style={
        'fontSize': '10px', 'fontWeight': 'bold', 'letterSpacing': '1.2px',
        'padding': '3px 8px', 'margin': '0 3px',
        'borderRadius': '4px',
        'border': f'1px solid {color if active else BDR}',
        'color': color if active else DIM,
        'background': CARD if active else BG,
    })


def _telcard(label, value, unit='', color=FG):
    return html.Div([
        html.Div(label, style=_LBL_S),
        html.Div(f'{value} {unit}'.strip(), style={**_VAL_S, 'color': color}),
    ], style=_CARD_S)


def _section(title, body):
    return html.Div([
        html.Div(title, style={**_LBL_S, 'borderBottom': f'1px solid {BDR}',
                               'paddingBottom': '5px', 'marginBottom': '7px'}),
        *body,
    ], style=_CARD_S)


# ── layout ────────────────────────────────────────────────────────────────
app.layout = html.Div([
    dcc.Interval(id='tick', interval=500, n_intervals=0),
    dcc.Store(id='_noop'),

    # ── header ──
    html.Div([
        html.Div([
            html.Span('◆ MARS ROVER — PYTHON DASH',
                      style={'color': BLUE, 'fontWeight': 'bold', 'fontSize': '15px',
                             'marginRight': '10px'}),
            html.Span('localhost:8050',
                      style={'color': DIM, 'fontSize': '12px'}),
        ], style={'display': 'flex', 'alignItems': 'center'}),

        html.Div([
            html.Span(id='uptime-lbl',
                      style={'color': DIM, 'fontSize': '11px', 'marginRight': '8px'}),
            html.Span('| 500ms poll', style={'color': DIM, 'fontSize': '11px'}),
        ]),

        html.Div(id='header-badges',
                 style={'display': 'flex', 'alignItems': 'center'}),
    ], style={
        'display': 'flex', 'justifyContent': 'space-between', 'alignItems': 'center',
        'background': CARD, 'border': f'1px solid {BDR}',
        'padding': '10px 16px', 'marginBottom': '10px', 'borderRadius': '5px',
    }),

    # ── 3-column body ──
    html.Div([

        # left: telemetry cards
        html.Div(id='tele-col', style={'width': '195px', 'flexShrink': '0'}),

        # centre: charts
        html.Div([
            html.Div([
                html.Div('SENSOR TIME-SERIES — temp / lidar / altitude (plotly dual-axis)',
                         style=_LBL_S),
                dcc.Graph(id='chart-sensor',
                          config={'displayModeBar': False},
                          style={'height': '225px'}),
            ], style=_CARD_S),
            html.Div([
                html.Div('POWER USAGE — budget threshold line',
                         style=_LBL_S),
                dcc.Graph(id='chart-power',
                          config={'displayModeBar': False},
                          style={'height': '200px'}),
            ], style=_CARD_S),
        ], style={'flex': '1', 'margin': '0 10px'}),

        # right: health / events / earth commands
        html.Div([
            html.Div(id='panel-health'),
            html.Div(id='panel-events'),
            html.Div(id='panel-earth'),
        ], style={'width': '220px', 'flexShrink': '0'}),

    ], style={'display': 'flex', 'alignItems': 'flex-start',
              'marginBottom': '10px'}),

    # ── navigation map ──
    html.Div([
        html.Div(
            'NAVIGATION MAP — waypoint scatter'
            ' (plotly, zoom/pan enabled, hover shows temp + lidar per waypoint)',
            style=_LBL_S,
        ),
        dcc.Graph(id='nav-map',
                  config={'scrollZoom': True, 'displayModeBar': True,
                          'modeBarButtonsToRemove': ['autoScale2d', 'lasso2d', 'select2d']},
                  style={'height': '290px'}),
    ], style={**_CARD_S, 'marginBottom': '10px'}),

    # ── status log ──
    html.Div([
        html.Div(
            'STATUS LOG — StatusReport_t from xStatusQueue, drained by Comms TX',
            style=_LBL_S,
        ),
        dash_table.DataTable(
            id='status-log',
            columns=[
                {'name': 'TIME',    'id': 'time',    'type': 'text'},
                {'name': 'TASK',    'id': 'task',    'type': 'text'},
                {'name': 'LEVEL',   'id': 'level',   'type': 'text'},
                {'name': 'MESSAGE', 'id': 'message', 'type': 'text'},
            ],
            data=[],
            page_size=6,
            sort_action='native',
            style_as_list_view=True,
            style_table={'overflowX': 'auto'},
            style_header={
                'background': BG, 'color': DIM,
                'fontFamily': FONT, 'fontSize': '10px', 'letterSpacing': '1px',
                'fontWeight': 'bold', 'textTransform': 'uppercase',
                'border': f'1px solid {BDR}', 'borderBottom': f'1px solid {BLUE}',
            },
            style_cell={
                'background': CARD, 'color': FG,
                'fontFamily': FONT, 'fontSize': '12px',
                'textAlign': 'left', 'padding': '6px 12px',
                'border': f'1px solid {BDR}',
            },
            style_data_conditional=[
                {'if': {'filter_query': '{level} = "ERROR"'},   'color': RED},
                {'if': {'filter_query': '{level} = "FAULT"'},   'color': RED},
                {'if': {'filter_query': '{level} = "WARNING"'}, 'color': AMBER},
                {'if': {'filter_query': '{level} = "OK"'},      'color': GREEN},
            ],
        ),
    ], style={**_CARD_S, 'marginBottom': '10px'}),

    # ── command buttons ──
    html.Div([
        html.Div('COMMANDS', style=_LBL_S),
        html.Div([
            html.Div([html.Button('▲  FORWARD',  id='btn-fwd',   style=_BTN_S)],
                     style={'textAlign': 'center'}),
            html.Div([
                html.Button('◀  LEFT',   id='btn-left',  style=_BTN_S),
                html.Button('■  STOP',   id='btn-stop',  style=_BTN_S),
                html.Button('RIGHT  ▶',  id='btn-right', style=_BTN_S),
            ], style={'textAlign': 'center'}),
            html.Div([html.Button('▼  BACKWARD', id='btn-bwd',   style=_BTN_S)],
                     style={'textAlign': 'center'}),
            html.Div([html.Button('⚠  EMERGENCY STOP', id='btn-estop', style=_ESTOP_S)],
                     style={'textAlign': 'center'}),
        ]),
    ], style=_CARD_S),

], style={
    'background': BG, 'color': FG, 'fontFamily': FONT,
    'padding': '12px 16px', 'minHeight': '100vh',
})


def _ago(t, now):
    if t is None: return 'never'
    d = now - t
    if d < 5:  return '< 5s'
    if d < 60: return f'{d:.0f}s ago'
    return 'stale'


# ═══════════════════════════════════════════════════ main callback ════════
@app.callback(
    [
        Output('uptime-lbl',     'children'),
        Output('header-badges',  'children'),
        Output('tele-col',       'children'),
        Output('chart-sensor',   'figure'),
        Output('chart-power',    'figure'),
        Output('panel-health',   'children'),
        Output('panel-events',   'children'),
        Output('panel-earth',    'children'),
        Output('nav-map',        'figure'),
        Output('status-log',     'data'),
    ],
    Input('tick', 'n_intervals'),
)
def _refresh(_n):
    elapsed_now = time.time() - _t0

    # snapshot state under lock
    with _lock:
        x, y, hdg     = _x, _y, _hdg
        path, obs     = _path, _obs
        orig, temp    = _orig, _temp
        lidar, alt    = _lidar, _alt
        power         = _power
        wp_count      = _wp_count
        events        = dict(_events)
        tasks         = {k: list(v) for k, v in _tasks.items()}
        wps           = list(_waypoints)
        ecmds         = list(_earth_cmds)
        slog          = list(_status_log)
        ts_t_s        = list(_ts['t_s'])
        ts_temp       = list(_ts['temp'])
        ts_lidar      = list(_ts['lidar'])
        ts_alt        = list(_ts['alt'])
        ts_t_p        = list(_ts['t_p'])
        ts_power      = list(_ts['power'])
        last_seen     = dict(_last_seen)
        mct_sent      = list(_mct_sent)

    # ── uptime ──
    up = int(time.time() - _t0)
    h, r = divmod(up, 3600); m2, s = divmod(r, 60)
    uptime_str = f'UPTIME {h:02d}:{m2:02d}:{s:02d}'

    # ── badges ──
    badges = [
        _badge('OBSTACLE',     events['OBSTACLE'],     RED),
        _badge('LOW POWER',    events['LOW_POWER'],    AMBER),
        _badge('COMMS LOST',   events['COMMS_LOST'],   AMBER),
        _badge('SENSOR FAULT', events['SENSOR_FAULT'], AMBER),
        _badge('MOTOR FAULT',  events['MOTOR_FAULT'],  AMBER),
        _badge('EMERG STOP',   events['EMERG_STOP'],   RED),
    ]

    # ── telemetry cards ──
    hdg_str = {0: 'N (0°)', 90: 'E (90°)',
               180: 'S (180°)', 270: 'W (270°)'}.get(hdg, f'{hdg}°')

    def _obs_c(v): return RED if v < 50 else AMBER if v < 100 else FG
    def _pwr_c(v): return RED if v > 90 else AMBER if v > 70 else GREEN

    alt_str = (f'+{alt}' if alt >= 0 else str(alt))
    tele = [
        _telcard('Temperature',  f'{temp:.1f}',  '°C',    AMBER),
        _telcard('Lidar Dist',   lidar,          'cm',    _obs_c(lidar)),
        _telcard('Altitude',     alt_str,        'cm'),
        _telcard('Power Budget', f'{power} / 100', '',    _pwr_c(power)),
        _telcard('Heading',      hdg_str,         '',     BLUE),
        _telcard('From Origin',  f'{orig:.0f}',  'cm',   BLUE),
        _telcard('Path Total',   path,           'cm',   GREEN),
        _telcard('Waypoints',    f'{wp_count % MAX_WP} / {MAX_WP}'),
    ]

    # ── sensor time-series (dual y-axis) ──
    fig_s = make_subplots(specs=[[{'secondary_y': True}]])
    if ts_t_s:
        fig_s.add_trace(go.Scatter(
            x=ts_t_s, y=ts_temp,  name='temp',
            line=dict(color=AMBER, width=1.5)), secondary_y=False)
        fig_s.add_trace(go.Scatter(
            x=ts_t_s, y=ts_lidar, name='lidar',
            line=dict(color=BLUE,  width=1.5)), secondary_y=True)
        fig_s.add_trace(go.Scatter(
            x=ts_t_s, y=ts_alt,   name='alt',
            line=dict(color=GREEN, width=1.5, dash='dot')), secondary_y=True)
        # obstacle threshold on lidar axis
        if ts_t_s:
            fig_s.add_trace(go.Scatter(
                x=[ts_t_s[0], ts_t_s[-1]], y=[50, 50],
                name='obstacle', mode='lines',
                line=dict(color=RED, width=1, dash='dash'),
                showlegend=True), secondary_y=True)
    fig_s.update_layout(**PLOT_LAYOUT)
    fig_s.update_yaxes(title_text='°C', secondary_y=False, **AXIS)
    fig_s.update_yaxes(title_text='cm', secondary_y=True,  **AXIS)
    fig_s.update_xaxes(title_text='elapsed s', **AXIS)

    # ── power usage chart ──
    fig_p = go.Figure()
    if ts_t_p:
        fig_p.add_trace(go.Bar(
            x=ts_t_p, y=ts_power, name='power',
            marker_color=BLUE, opacity=0.75))
        fig_p.add_trace(go.Scatter(
            x=[ts_t_p[0], ts_t_p[-1]], y=[100, 100],
            name='budget', mode='lines',
            line=dict(color=RED, width=1.5, dash='dash')))
    fig_p.update_layout(
        **PLOT_LAYOUT,
        yaxis=dict(range=[50, 115], title_text='units', **AXIS),
        xaxis=dict(title_text='elapsed s', **AXIS),
        bargap=0.1,
    )

    # ── task health panel ──
    sc = {'ALIVE': GREEN, 'WARN': AMBER, 'FAULT': RED}
    health_rows = []
    for name, (pri, st) in tasks.items():
        ago_str = _ago(last_seen.get(name), elapsed_now)
        health_rows.append(html.Div([
            html.Div([
                html.Span('● ', style={'color': sc.get(st, DIM)}),
                html.Span(name, style={'color': FG, 'fontSize': '12px'}),
                html.Span(f' P{pri} {st}',
                          style={'color': sc.get(st, DIM), 'fontSize': '11px',
                                 'float': 'right'}),
            ], style={'overflow': 'hidden'}),
            html.Div(f'  ↳ last seen: {ago_str}',
                     style={'color': DIM, 'fontSize': '9px',
                            'paddingLeft': '12px', 'marginBottom': '5px'}),
        ]))
    panel_health = _section('Task Health', health_rows)

    # ── event flags panel ──
    ev_c = dict(OBSTACLE=RED, LOW_POWER=AMBER, COMMS_LOST=AMBER,
                SENSOR_FAULT=AMBER, MOTOR_FAULT=AMBER, EMERG_STOP=RED)
    ev_rows = [
        html.Div([
            html.Span(k.replace('_', ' '),
                      style={'fontSize': '11px', 'letterSpacing': '1px',
                             'color': ev_c.get(k, AMBER) if active else DIM}),
            html.Span(' SET' if active else '',
                      style={'fontSize': '10px', 'fontWeight': 'bold',
                             'color': ev_c.get(k, AMBER), 'float': 'right'}),
        ], style={'marginBottom': '4px', 'overflow': 'hidden'})
        for k, active in events.items()
    ]
    panel_events = _section('Event Flags', ev_rows)

    # ── commands panel (MCT sent + Earth RX) ──
    mct_rows = [
        html.Div([
            html.Span(ec['time'],
                      style={'color': DIM, 'fontSize': '10px', 'marginRight': '6px'}),
            html.Span(f"▶ {ec['cmd']}",
                      style={'color': GREEN, 'fontSize': '11px', 'fontWeight': 'bold'}),
        ], style={'marginBottom': '3px'})
        for ec in reversed(mct_sent)
    ] or [html.Div('—', style={'color': DIM, 'fontSize': '11px'})]

    earth_rows = [
        html.Div([
            html.Span(ec['time'],
                      style={'color': DIM, 'fontSize': '10px', 'marginRight': '6px'}),
            html.Span(f"{ec['cmd']} {ec['speed']}%",
                      style={'color': BLUE, 'fontSize': '11px', 'fontWeight': 'bold'}),
        ], style={'marginBottom': '3px'})
        for ec in reversed(list(ecmds))
    ] or [html.Div('—', style={'color': DIM, 'fontSize': '11px'})]

    panel_earth = _section('Commands', [
        html.Div('MCT SENT', style={**_LBL_S, 'fontSize': '9px', 'marginBottom': '3px'}),
        html.Div(mct_rows, style={'maxHeight': '90px', 'overflowY': 'auto',
                                  'marginBottom': '8px'}),
        html.Div('EARTH RX', style={**_LBL_S, 'fontSize': '9px', 'marginBottom': '3px',
                                    'borderTop': f'1px solid {BDR}', 'paddingTop': '6px'}),
        html.Div(earth_rows, style={'maxHeight': '90px', 'overflowY': 'auto'}),
    ])

    # ── navigation map ──
    fig_m = go.Figure()
    if wps:
        fig_m.add_trace(go.Scatter(
            x=[w['x'] for w in wps],
            y=[w['y'] for w in wps],
            mode='markers+lines',
            marker=dict(color=BLUE, size=6,
                        line=dict(color='white', width=0.5)),
            line=dict(color=BLUE, width=1),
            text=[f"WP #{w['n']}<br>{w['temp']:.1f} °C<br>dist: {w['dist']}cm"
                  for w in wps],
            hoverinfo='text', name='waypoints',
        ))
    # rover marker — triangle points in the direction of travel
    _HDG_SYM = {0: 'triangle-up', 90: 'triangle-right',
                180: 'triangle-down', 270: 'triangle-left'}
    fig_m.add_trace(go.Scatter(
        x=[x], y=[y], mode='markers',
        marker=dict(symbol=_HDG_SYM.get(hdg, 'triangle-up'),
                    color=GREEN, size=14,
                    line=dict(color='white', width=1)),
        hovertext=f'Rover ({x}, {y}) cm  Heading:{hdg}°', hoverinfo='text',
        name='rover',
    ))
    # origin marker
    fig_m.add_trace(go.Scatter(
        x=[0], y=[0], mode='markers',
        marker=dict(symbol='circle', color=GREEN, size=9,
                    line=dict(color='white', width=1)),
        name='origin',
    ))
    fig_m.update_layout(**PLOT_LAYOUT)
    fig_m.update_layout(
        xaxis=dict(title_text='X (cm East) →', scaleanchor='y',
                   scaleratio=1, **AXIS),
        yaxis=dict(title_text='Y (cm North) ↑', **AXIS),
        dragmode='pan',
        margin=dict(l=44, r=16, t=8, b=52),
    )
    wp_n = wp_count % MAX_WP
    fig_m.add_annotation(
        text=(f'WP {wp_n}/{MAX_WP}  |  scale 1cm:1px  |  zoom/pan enabled via plotly'
              '  |  hover any waypoint for sensor snapshot'),
        xref='paper', yref='paper', x=0, y=0,
        xanchor='left', yanchor='top', yshift=-26,
        font=dict(size=9, color=DIM), showarrow=False,
    )

    # ── status log data ──
    log_data = list(reversed(slog))

    return (uptime_str, badges, tele,
            fig_s, fig_p,
            panel_health, panel_events, panel_earth,
            fig_m, log_data)


# ═══════════════════════════════════════════════════ command callback ═════
@app.callback(
    Output('_noop', 'data'),
    [Input('btn-fwd',   'n_clicks'),
     Input('btn-bwd',   'n_clicks'),
     Input('btn-left',  'n_clicks'),
     Input('btn-right', 'n_clicks'),
     Input('btn-stop',  'n_clicks'),
     Input('btn-estop', 'n_clicks')],
    prevent_initial_call=True,
)
def _cmd_cb(*_):
    _map = {
        'btn-fwd':   'forward 50 1000',
        'btn-bwd':   'backward 50 1000',
        'btn-left':  'left',
        'btn-right': 'right',
        'btn-stop':  'stop',
        'btn-estop': 'estop',
    }
    if ctx.triggered_id in _map:
        _send(_map[ctx.triggered_id])
    return dash.no_update


# ═══════════════════════════════════════════════════ entry point ══════════
if __name__ == '__main__':
    _start()
    app.run(debug=False, host='0.0.0.0', port=8050)
