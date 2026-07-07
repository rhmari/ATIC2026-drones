"""
╔══════════════════════════════════════════════════════════════╗
║   Drone Formal Rulebook — 3D Animated Compliance Monitor    ║
║   ATIC 2026 · ETH Zürich          v3 — Mission Planning     ║
║                                                              ║
║   Two trajectories for the same delivery mission:           ║
║     NAIVE  — direct route, rule-unaware (animated, red)     ║
║     OPT    — STL-guided optimiser output     (static, green)║
║                                                              ║
║   7 STL rules · 5 tiers · EASA (EU) 2019/947                ║
║                                                              ║
║   Dependencies:  pip install plotly numpy scipy             ║
║   Run:           python drone_simulation.py                  ║
╚══════════════════════════════════════════════════════════════╝
"""

import json, urllib.parse, urllib.request
import numpy as np
import plotly.graph_objects as go

try:
    from scipy.optimize import minimize as sp_minimize
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False
    print("\n  [!] scipy not found. Run:  pip install scipy\n"
          "      Showing naive trajectory only.\n")


# ══════════════════════════════════════════════════════════════════
#  1.  COORDINATE SYSTEM  (WGS84 → local metres)
# ══════════════════════════════════════════════════════════════════

BBOX    = (47.372, 8.537, 47.383, 8.556)
LAT0, LON0 = BBOX[0], BBOX[1]
LAT_MID    = (BBOX[0] + BBOX[2]) / 2.0
M_PER_DEG_LAT = 111_320.0
M_PER_DEG_LON = np.cos(np.radians(LAT_MID)) * M_PER_DEG_LAT

def to_xy(lat, lon):
    return (lon - LON0) * M_PER_DEG_LON, (lat - LAT0) * M_PER_DEG_LAT

ENV = dict(
    x=(BBOX[3]-BBOX[1])*M_PER_DEG_LON,   # ≈ 1 434 m
    y=(BBOX[2]-BBOX[0])*M_PER_DEG_LAT,   # ≈ 1 224 m
    z=250,
)


# ══════════════════════════════════════════════════════════════════
#  2.  RULEBOOK CONFIGURATION
# ══════════════════════════════════════════════════════════════════

ALTITUDE_MAX = 120.0    # REQ-ZONE-03: 120 m AGL ceiling

_NFZ_LATLON = [
    (47.3757, 8.5420,  80, 150, "UniversitätsSpital Helipad"),
    (47.3769, 8.5477,  60, 150, "ETH Zürich Campus"),
    (47.3779, 8.5402,  70, 150, "Zürich Hauptbahnhof"),
]
NO_FLY_ZONES = [(to_xy(la,lo)[0], to_xy(la,lo)[1], r, h, lb)
                for la, lo, r, h, lb in _NFZ_LATLON]

GEOFENCE = dict(x_min=0, x_max=ENV['x'], y_min=0, y_max=ENV['y'])

_CROWD_LATLON = [
    (47.3765, 8.5435, 100.0, "Niederdorf – public assembly"),
    (47.3748, 8.5415,  80.0, "Central district – crowd zone"),
]
CROWD_ZONES = [(to_xy(la,lo)[0], to_xy(la,lo)[1], s, lb)
               for la, lo, s, lb in _CROWD_LATLON]

SPEED_MAX    = 19.0     # REQ-EQP-07: C1-class speed cap (m/s)
MISSION_TIME = 160.0    # seconds for full T-step mission
BATT_START, BATT_END, BATT_WARN = 100.0, 14.0, 20.0
NOISE_LIMIT, NOISE_BASE, NOISE_SLOPE = 70.0, 50.0, 1.2

TRAIL_LEN  = 60
FRAME_STEP = 4
FRAME_DUR  = 45


# ══════════════════════════════════════════════════════════════════
#  3.  OSM BUILDING FETCH & PARSE
# ══════════════════════════════════════════════════════════════════

def fetch_buildings(bbox, timeout=25):
    la0, lo0, la1, lo1 = bbox
    q = (f"[out:json][timeout:{timeout}];"
         f"(way[\"building\"]({la0},{lo0},{la1},{lo1}););out geom;")
    data = urllib.parse.urlencode({'data': q}).encode()
    try:
        req = urllib.request.Request(
            "https://overpass-api.de/api/interpreter", data=data,
            headers={'User-Agent': 'ATIC2026-DroneSimulation/1.0'})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode()).get('elements', [])
    except Exception as exc:
        print(f"  [WARNING] Building fetch failed: {exc}"); return []

def parse_buildings(elements, default_h=12.0):
    result = []
    for el in elements:
        if el.get('type') != 'way': continue
        geom = el.get('geometry', [])
        if len(geom) < 4: continue
        tags = el.get('tags', {})
        try:    h = float(tags.get('height', 0) or 0)
        except: h = 0.0
        if h < 1.0:
            try:    h = float(tags.get('building:levels', 0) or 0) * 3.2
            except: h = 0.0
        if h < 3.0: h = default_h
        xs, ys = [], []
        for nd in geom:
            x, y = to_xy(nd['lat'], nd['lon'])
            xs.append(x); ys.append(y)
        if len(xs) > 1 and abs(xs[0]-xs[-1]) < 0.1 and abs(ys[0]-ys[-1]) < 0.1:
            xs, ys = xs[:-1], ys[:-1]
        if len(xs) >= 3: result.append((xs, ys, h))
    return result

def extrude_polygon(xs, ys, h):
    n  = len(xs)
    vx = list(xs)+list(xs); vy = list(ys)+list(ys)
    vz = [0.0]*n + [float(h)]*n
    faces = []
    for k in range(1,n-1): faces.append([0,k,k+1])
    for k in range(1,n-1): faces.append([n,n+k+1,n+k])
    for k in range(n):
        k1=(k+1)%n
        faces.append([k,k1,n+k1]); faces.append([k,n+k1,n+k])
    return (np.array(vx), np.array(vy), np.array(vz)), np.array(faces)

def build_city_mesh(buildings):
    ax,ay,az,ai,aj,ak = [],[],[],[],[],[]
    off = 0
    for xs,ys,h in buildings:
        (vx,vy,vz), faces = extrude_polygon(xs, ys, h)
        ax.extend(vx); ay.extend(vy); az.extend(vz)
        ai.extend(faces[:,0]+off); aj.extend(faces[:,1]+off); ak.extend(faces[:,2]+off)
        off += len(vx)
    if not ax: return None
    return go.Mesh3d(x=ax,y=ay,z=az,i=ai,j=aj,k=ak,
        color='#4a5568', opacity=0.80, name='Zürich Buildings', showlegend=True,
        hoverinfo='skip', flatshading=True,
        lighting=dict(ambient=0.7,diffuse=0.8,roughness=0.5),
        lightposition=dict(x=700,y=400,z=1500))

print('\n  Fetching Zürich building data from OpenStreetMap …')
_elements  = fetch_buildings(BBOX)
_buildings = parse_buildings(_elements)
print(f'  Loaded {len(_buildings)} buildings.\n')


# ══════════════════════════════════════════════════════════════════
#  4.  TRAJECTORY HELPER  (Catmull-Rom, pure numpy)
# ══════════════════════════════════════════════════════════════════

def catmull_rom(waypoints, n_points):
    P = np.array(waypoints, dtype=float)
    n = len(P); segs = n-1; pps = n_points//segs
    result = []
    for i in range(segs):
        p0=P[max(i-1,0)]; p1=P[i]; p2=P[i+1]; p3=P[min(i+2,n-1)]
        count = pps if i < segs-1 else n_points-len(result)
        for t in np.linspace(0, 1, count, endpoint=(i==segs-1)):
            t2,t3=t*t,t*t*t
            result.append(0.5*(2*p1+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3))
    return np.array(result)

T = 400


# ══════════════════════════════════════════════════════════════════
#  5.  MISSION  (parcel delivery: start → target → return)
# ══════════════════════════════════════════════════════════════════

_START_LAT,    _START_LON,    _START_ALT    = 47.3725, 8.538,   25.0
_DELIVERY_LAT, _DELIVERY_LON, _DELIVERY_ALT = 47.3818, 8.5515,  25.0

START_WP    = [*to_xy(_START_LAT,    _START_LON),    _START_ALT]
DELIVERY_WP = [*to_xy(_DELIVERY_LAT, _DELIVERY_LON), _DELIVERY_ALT]
DELIVERY_RADIUS = 30.0   # ground zone radius (m)

# ── NAIVE trajectory ──────────────────────────────────────────────
#   Rule-unaware direct route: goes straight through NFZ zones
#   and climbs above the altitude ceiling.  This is what a planner
#   that ignores the rulebook would produce.
_NAIVE_WPS = [
    START_WP,
    [*to_xy(47.3757, 8.5420), 130.0],   # ← through USZ Helipad + above ceiling
    DELIVERY_WP,
    [*to_xy(47.3769, 8.5477), 100.0],   # ← through ETH Campus NFZ on return
    START_WP,
]
traj_naive = catmull_rom(_NAIVE_WPS, T)

# ── OPTIMIZER settings ────────────────────────────────────────────
N_FREE  = 3       # free intermediate waypoints per leg (3 out + 3 back = 18 DOF)
T_OPT   = 80      # coarser resolution during optimisation (faster per-call)
LAMBDA  = 12.0    # STL violation penalty weight


# ══════════════════════════════════════════════════════════════════
#  6.  SIGNALS  (speed, battery, noise from trajectory)
# ══════════════════════════════════════════════════════════════════

def compute_signals(traj):
    """Compute all derived signals for any-length trajectory."""
    n   = len(traj)
    dt  = MISSION_TIME / n
    dp  = np.diff(traj, axis=0, prepend=traj[[0]])
    spd = np.linalg.norm(dp, axis=1) / dt
    spd[0] = spd[1]                            # fix prepend artefact
    bat = np.linspace(BATT_START, BATT_END, n)
    nse = NOISE_BASE + NOISE_SLOPE * spd
    return dict(speed=spd, battery=bat, noise=nse)


# ══════════════════════════════════════════════════════════════════
#  7.  RULES LIST  (rob_fn works for any trajectory length)
# ══════════════════════════════════════════════════════════════════

def _rob_crowd(traj, sig):
    n=len(traj); x,y=traj[:,0],traj[:,1]; rob=np.full(n,np.inf)
    for cx,cy,s,_ in CROWD_ZONES:
        rob=np.minimum(rob, np.sqrt((x-cx)**2+(y-cy)**2)-s)
    return rob

def _rob_geofence(traj, sig):
    x,y=traj[:,0],traj[:,1]; f=GEOFENCE
    return np.minimum(np.minimum(x-f['x_min'],f['x_max']-x),
                      np.minimum(y-f['y_min'],f['y_max']-y))

def _rob_altitude(traj, sig):
    return ALTITUDE_MAX - traj[:,2]

def _rob_nfz(traj, sig):
    n=len(traj); x,y,z=traj[:,0],traj[:,1],traj[:,2]; rob=np.full(n,np.inf)
    for cx,cy,r,h,_ in NO_FLY_ZONES:
        dist=np.sqrt((x-cx)**2+(y-cy)**2)
        rob=np.minimum(rob, np.where(z<=h, dist-r, np.inf))
    return rob

def _rob_speed(traj, sig):   return SPEED_MAX   - sig['speed']
def _rob_battery(traj, sig): return sig['battery'] - BATT_WARN
def _rob_noise(traj, sig):   return NOISE_LIMIT  - sig['noise']

RULES = [
    dict(id='REQ-PROX-01', tier=1, label='Crowd Standoff',
         short='Crowd', color='#c0392b', rob_fn=_rob_crowd),
    dict(id='REQ-PROX-06', tier=2, label='Geofence Containment',
         short='Geo',   color='#f39c12', rob_fn=_rob_geofence),
    dict(id='REQ-ZONE-03', tier=3, label='Altitude Ceiling (120 m)',
         short='Alt',   color='#e74c3c', rob_fn=_rob_altitude),
    dict(id='REQ-ZONE-04', tier=3, label='No-Fly Zone Access',
         short='NFZ',   color='#e67e22', rob_fn=_rob_nfz),
    dict(id='REQ-EQP-07',  tier=5, label=f'Max Speed (≤{SPEED_MAX:.0f} m/s)',
         short='Spd',   color='#9b59b6', rob_fn=_rob_speed),
    dict(id='REQ-EQP-03',  tier=5, label=f'Battery Warning (≥{BATT_WARN:.0f}%)',
         short='Bat',   color='#1abc9c', rob_fn=_rob_battery),
    dict(id='REQ-EQP-12',  tier=7, label=f'Noise Limit (≤{NOISE_LIMIT:.0f} dB)',
         short='Noise', color='#3498db', rob_fn=_rob_noise),
]

C_OK = '#2ecc71'
_RULE_TIER_ORDER = sorted(range(len(RULES)), key=lambda k: RULES[k]['tier'])


def monitor_for(traj):
    """
    Evaluate all RULES for `traj`.
    Returns (results, all_ok, colors) where:
      results  — list of {rob: ndarray, ok: bool ndarray} in RULES order
      all_ok   — bool ndarray, True when every rule passes
      colors   — list of hex strings, colour of highest-priority violation per step
    """
    sig     = compute_signals(traj)
    n       = len(traj)
    results = []
    for rule in RULES:
        rob = rule['rob_fn'](traj, sig)
        results.append(dict(rob=rob, ok=(rob >= 0.0)))

    all_ok = np.ones(n, dtype=bool)
    for rr in results:
        all_ok &= rr['ok']

    colors = []
    for i in range(n):
        c = C_OK
        for k in _RULE_TIER_ORDER:
            if not results[k]['ok'][i]:
                c = RULES[k]['color']
                break
        colors.append(c)

    return results, all_ok, colors


# ══════════════════════════════════════════════════════════════════
#  8.  STL-GUIDED TRAJECTORY OPTIMISER
# ══════════════════════════════════════════════════════════════════

def _traj_from_free(params, n_pts=T):
    """Reconstruct trajectory from flat (2·N_FREE·3,) parameter vector."""
    pts   = params.reshape(2*N_FREE, 3)
    out_p = pts[:N_FREE].tolist()
    ret_p = pts[N_FREE:].tolist()
    wps   = [START_WP] + out_p + [DELIVERY_WP] + ret_p + [START_WP]
    return catmull_rom(wps, n_pts)

def _opt_objective(params):
    traj_o  = _traj_from_free(params, T_OPT)
    sig_o   = compute_signals(traj_o)
    # Tier-8 objective: minimise path length (REQ mission efficiency)
    path_len = float(np.sum(np.linalg.norm(np.diff(traj_o, axis=0), axis=1)))
    # STL penalty: sum of violation depths across all rules and steps
    penalty  = 0.0
    for rule in RULES:
        rob      = rule['rob_fn'](traj_o, sig_o)
        penalty += float(np.sum(np.maximum(0.0, -rob)))
    return path_len + LAMBDA * penalty

# Initial guess: straight interpolation at cruise altitude
def _make_init():
    pts = np.zeros((2*N_FREE, 3))
    for k, a in enumerate(np.linspace(0, 1, N_FREE+2)[1:-1]):
        pts[k]        = (1-a)*np.array(START_WP)    + a*np.array(DELIVERY_WP)
        pts[k,   2]   = 70.0
        pts[N_FREE+k] = (1-a)*np.array(DELIVERY_WP) + a*np.array(START_WP)
        pts[N_FREE+k, 2] = 70.0
    return pts.flatten()

_bounds = []
for _ in range(2*N_FREE):
    _bounds.extend([(10, ENV['x']-10), (10, ENV['y']-10), (20, ALTITUDE_MAX-2)])

if SCIPY_OK:
    print('  Running STL-guided trajectory optimiser …')
    _opt = sp_minimize(
        _opt_objective, _make_init(),
        method='L-BFGS-B', bounds=_bounds,
        options=dict(maxiter=2000, ftol=1e-9, gtol=1e-6),
    )
    traj_opt = _traj_from_free(_opt.x, T)
    print(f'  Optimiser converged: {_opt.message}  ({_opt.nit} iterations)')
    # Final penalty at full resolution
    _fin_pen = sum(
        float(np.sum(np.maximum(0.0, -rule['rob_fn'](traj_opt, compute_signals(traj_opt)))))
        for rule in RULES
    )
    print(f'  Residual STL violation depth: {_fin_pen:.1f} m  '
          f'(0 = fully compliant)\n')
else:
    traj_opt = None


# ══════════════════════════════════════════════════════════════════
#  9.  RUN MONITOR ON BOTH TRAJECTORIES
# ══════════════════════════════════════════════════════════════════

naive_results, naive_all_ok, naive_colors = monitor_for(traj_naive)
rob_matrix_naive = np.column_stack([rr['rob'] for rr in naive_results])

if traj_opt is not None:
    opt_results, opt_all_ok, opt_colors = monitor_for(traj_opt)
    rob_matrix_opt = np.column_stack([rr['rob'] for rr in opt_results])
else:
    opt_results = opt_all_ok = opt_colors = rob_matrix_opt = None


# ══════════════════════════════════════════════════════════════════
#  10. SCENE HELPERS
# ══════════════════════════════════════════════════════════════════

def nfz_cylinder(cx, cy, r, h, name, n=60):
    th = np.linspace(0, 2*np.pi, n)
    TH, ZZ = np.meshgrid(th, [0.0, float(h)])
    return go.Surface(x=cx+r*np.cos(TH), y=cy+r*np.sin(TH), z=ZZ,
        colorscale=[[0,'#c0392b'],[1,'#c0392b']], showscale=False, opacity=0.35,
        name=name, showlegend=True,
        hovertemplate=f'<b>{name}</b><extra></extra>')

def nfz_disc(cx, cy, r, h, n=60):
    rv=np.linspace(0,r,12); th=np.linspace(0,2*np.pi,n)
    R,TH=np.meshgrid(rv,th)
    return go.Surface(x=cx+R*np.cos(TH), y=cy+R*np.sin(TH),
        z=np.full_like(R, float(h)),
        colorscale=[[0,'#c0392b'],[1,'#c0392b']], showscale=False, opacity=0.25,
        showlegend=False, hoverinfo='skip')

def crowd_cylinder(cx, cy, standoff, name, n=60):
    th=np.linspace(0,2*np.pi,n); TH,ZZ=np.meshgrid(th,[0.0,40.0])
    return go.Surface(x=cx+standoff*np.cos(TH), y=cy+standoff*np.sin(TH), z=ZZ,
        colorscale=[[0,'#16a085'],[1,'#16a085']], showscale=False, opacity=0.22,
        name=name, showlegend=True,
        hovertemplate=f'<b>{name}</b><extra></extra>')

def altitude_ceiling(x_max, y_max, z_ceil):
    X=np.array([[0,x_max],[0,x_max]],dtype=float)
    Y=np.array([[0,0],[y_max,y_max]],dtype=float)
    return go.Surface(x=X,y=Y,z=np.full_like(X,float(z_ceil)),
        colorscale=[[0,'#2980b9'],[1,'#2980b9']], showscale=False, opacity=0.15,
        name=f'Altitude Ceiling ({int(z_ceil)} m)', showlegend=True)

def geofence_edges(f, z_top, color='#9b59b6'):
    x0,x1=f['x_min'],f['x_max']; y0,y1=f['y_min'],f['y_max']
    corners=[(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]
    traces=[]
    for zv in (0.0, float(z_top)):
        traces.append(go.Scatter3d(
            x=[c[0] for c in corners], y=[c[1] for c in corners], z=[zv]*len(corners),
            mode='lines', line=dict(color=color,width=3,dash='dot'),
            name='Geofence' if zv==0 else '', showlegend=(zv==0)))
    for xi,yi in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]:
        traces.append(go.Scatter3d(x=[xi,xi],y=[yi,yi],z=[0.0,float(z_top)],
            mode='lines', line=dict(color=color,width=3,dash='dot'), showlegend=False))
    return traces

def delivery_disc(cx, cy, r=DELIVERY_RADIUS, n=40):
    rv=np.linspace(0,r,10); th=np.linspace(0,2*np.pi,n); R,TH=np.meshgrid(rv,th)
    return go.Surface(x=cx+R*np.cos(TH), y=cy+R*np.sin(TH), z=np.zeros_like(R),
        colorscale=[[0,'#f1c40f'],[1,'#f1c40f']], showscale=False, opacity=0.70,
        name='Delivery Zone', showlegend=True,
        hovertemplate='<b>Delivery Zone</b><extra></extra>')


# ══════════════════════════════════════════════════════════════════
#  11. BUILD STATIC SCENE
# ══════════════════════════════════════════════════════════════════

fig = go.Figure()

# ── City ──────────────────────────────────────────────────────────
city_mesh = build_city_mesh(_buildings)
if city_mesh: fig.add_trace(city_mesh)

# ── Constraint geometry ───────────────────────────────────────────
for tr in geofence_edges(GEOFENCE, ENV['z']): fig.add_trace(tr)
fig.add_trace(altitude_ceiling(ENV['x'], ENV['y'], ALTITUDE_MAX))
for cx,cy,r,h,lb in NO_FLY_ZONES:
    fig.add_trace(nfz_cylinder(cx,cy,r,h,lb))
    fig.add_trace(nfz_disc(cx,cy,r,h))
for cx,cy,s,lb in CROWD_ZONES:
    fig.add_trace(crowd_cylinder(cx,cy,s,lb))

# ── Delivery zone ─────────────────────────────────────────────────
fig.add_trace(delivery_disc(DELIVERY_WP[0], DELIVERY_WP[1]))
fig.add_trace(go.Scatter3d(
    x=[DELIVERY_WP[0]], y=[DELIVERY_WP[1]], z=[35.0],
    mode='markers+text', text=['  Delivery'],
    textfont=dict(color='#f1c40f', size=13),
    marker=dict(size=13, color='#f1c40f', symbol='diamond'),
    name='Delivery Target', showlegend=True,
))

# ── OPTIMISED trajectory (static, full path coloured) ─────────────
if traj_opt is not None:
    fig.add_trace(go.Scatter3d(
        x=traj_opt[:,0], y=traj_opt[:,1], z=traj_opt[:,2],
        mode='markers',
        marker=dict(size=3.0, color=opt_colors, opacity=0.90),
        name='Optimised path', showlegend=True,
        customdata=rob_matrix_opt,
        hovertemplate=(
            '<b>OPT (%{x:.0f} m, %{y:.0f} m, %{z:.0f} m)</b><br>' +
            ''.join(f'{RULES[j]["id"]}: %{{customdata[{j}]:+.1f}}<br>'
                    for j in range(len(RULES))) +
            '<extra></extra>'
        ),
    ))

# ── NAIVE ghost path (faint) ──────────────────────────────────────
fig.add_trace(go.Scatter3d(
    x=traj_naive[:,0], y=traj_naive[:,1], z=traj_naive[:,2],
    mode='lines',
    line=dict(color='rgba(255,255,255,0.10)', width=2),
    showlegend=False, hoverinfo='skip',
))

# ── Start / End / Delivery markers ────────────────────────────────
fig.add_trace(go.Scatter3d(
    x=[START_WP[0]], y=[START_WP[1]], z=[START_WP[2]],
    mode='markers+text', text=['  Home'],
    textfont=dict(color='white',size=12),
    marker=dict(size=10, color='white', symbol='circle'), name='Home',
))

# ── Legend: compliance colours ────────────────────────────────────
fig.add_trace(go.Scatter3d(x=[None],y=[None],z=[None], mode='markers',
    marker=dict(size=9,color=C_OK), name='Compliant', showlegend=True))
for rule in RULES:
    fig.add_trace(go.Scatter3d(x=[None],y=[None],z=[None], mode='markers',
        marker=dict(size=9,color=rule['color']),
        name=f"[T{rule['tier']}] {rule['id']}: {rule['label']}",
        showlegend=True))

# ── Animated traces (NAIVE, always last two indices) ──────────────
n_static = len(fig.data)

_hover_rows = ''.join(
    f'{RULES[j]["id"]}: %{{customdata[{j}]:+.1f}}<br>'
    for j in range(len(RULES))
)
_hover_tmpl = ('<b>NAIVE (%{x:.0f} m, %{y:.0f} m, %{z:.0f} m)</b><br>'
               + _hover_rows + '<extra></extra>')

fig.add_trace(go.Scatter3d(                         # trail
    x=[traj_naive[0,0]], y=[traj_naive[0,1]], z=[traj_naive[0,2]],
    mode='markers',
    marker=dict(size=3.5, color=[naive_colors[0]], opacity=0.95),
    showlegend=False,
    customdata=rob_matrix_naive[[0]],
    hovertemplate=_hover_tmpl,
))
fig.add_trace(go.Scatter3d(                         # drone dot
    x=[traj_naive[0,0]], y=[traj_naive[0,1]], z=[traj_naive[0,2]],
    mode='markers',
    marker=dict(size=14, color=naive_colors[0], symbol='circle',
                line=dict(color='white',width=2)),
    name='Naive drone', showlegend=True,
))


# ══════════════════════════════════════════════════════════════════
#  12. ANIMATION FRAMES  (naive trajectory)
# ══════════════════════════════════════════════════════════════════

def _status(i):
    parts = [f"{r['short']} {'✓' if naive_results[k]['ok'][i] else '✗'}"
             for k, r in enumerate(RULES)]
    pct   = 100 * naive_all_ok[:i+1].mean()
    return (f'NAIVE mission — Step {i+1}/{T}  |  '
            f'{" · ".join(parts)}  |  Compliant: {pct:.0f}%')

frame_steps = list(range(0, T, FRAME_STEP))
frames      = []

for fi in frame_steps:
    t0    = max(0, fi - TRAIL_LEN)
    sl    = slice(t0, fi+1)
    n_tr  = fi + 1 - t0
    sizes = np.linspace(1.5, 4.5, n_tr).tolist()

    frames.append(go.Frame(
        name=str(fi),
        traces=[n_static, n_static+1],
        data=[
            go.Scatter3d(
                x=traj_naive[sl,0].tolist(),
                y=traj_naive[sl,1].tolist(),
                z=traj_naive[sl,2].tolist(),
                mode='markers',
                marker=dict(size=sizes, color=naive_colors[t0:fi+1], opacity=0.92),
                customdata=rob_matrix_naive[sl],
                hovertemplate=_hover_tmpl,
            ),
            go.Scatter3d(
                x=[traj_naive[fi,0]], y=[traj_naive[fi,1]], z=[traj_naive[fi,2]],
                mode='markers',
                marker=dict(size=14, color=naive_colors[fi], symbol='circle',
                            line=dict(color='white',width=2)),
            ),
        ],
        layout=go.Layout(title=dict(text=_status(fi))),
    ))

fig.frames = frames


# ══════════════════════════════════════════════════════════════════
#  13. LAYOUT
# ══════════════════════════════════════════════════════════════════

fig.update_layout(
    title=dict(
        text='Zürich Drone Delivery — STL Rulebook Monitor & Trajectory Optimiser · ATIC 2026',
        font=dict(size=13,color='white'), x=0.5, xanchor='center',
    ),
    paper_bgcolor='#0d1117',
    scene=dict(
        xaxis=dict(title='East (m)',     range=[0,ENV['x']],
                   backgroundcolor='#161b22',gridcolor='#30363d',color='#8b949e'),
        yaxis=dict(title='North (m)',    range=[0,ENV['y']],
                   backgroundcolor='#161b22',gridcolor='#30363d',color='#8b949e'),
        zaxis=dict(title='Altitude (m)', range=[0,ENV['z']],
                   backgroundcolor='#161b22',gridcolor='#30363d',color='#8b949e'),
        bgcolor='#161b22', aspectmode='manual',
        aspectratio=dict(x=1, y=ENV['y']/ENV['x'], z=0.25),
        camera=dict(eye=dict(x=1.6,y=-1.6,z=1.0)),
    ),
    legend=dict(bgcolor='rgba(13,17,23,0.88)', bordercolor='#30363d', borderwidth=1,
                font=dict(color='white',size=10), x=0.01, y=0.99,
                xanchor='left', yanchor='top'),
    updatemenus=[dict(
        type='buttons', showactive=False,
        bgcolor='#161b22', bordercolor='#30363d', font=dict(color='white'),
        x=0.5, xanchor='center', y=-0.08, yanchor='top',
        buttons=[
            dict(label='▶  Play (naive)',  method='animate',
                 args=[None, dict(frame=dict(duration=FRAME_DUR, redraw=True),
                                  fromcurrent=True, transition=dict(duration=0),
                                  mode='immediate')]),
            dict(label='⏸  Pause', method='animate',
                 args=[[None], dict(frame=dict(duration=0, redraw=False),
                                    mode='immediate', transition=dict(duration=0))]),
        ],
    )],
    sliders=[dict(
        active=0, x=0.05, y=-0.02, len=0.90,
        bgcolor='#161b22', bordercolor='#30363d',
        tickcolor='#8b949e', font=dict(color='#8b949e',size=10),
        currentvalue=dict(prefix='Step: ', visible=True, xanchor='center',
                          font=dict(color='white',size=12)),
        transition=dict(duration=0),
        steps=[
            dict(method='animate', label=str(fi),
                 args=[[str(fi)], dict(frame=dict(duration=0,redraw=True),
                                       mode='immediate',transition=dict(duration=0))])
            for fi in frame_steps
        ],
    )],
    margin=dict(l=0,r=0,t=50,b=100),
)


# ══════════════════════════════════════════════════════════════════
#  14. COMPLIANCE REPORT  (naive vs. optimised, side-by-side)
# ══════════════════════════════════════════════════════════════════

_TIER_LABEL = {
    1: 'Tier 1 - Safety of Humans',
    2: 'Tier 2 - Safety of Property',
    3: 'Tier 3 - Major Infractions',
    5: 'Tier 5 - Control',
    7: 'Tier 7 - Noise / Comfort',
}

def _fmt_rule(results, idx):
    rr  = results[idx]
    fin = rr['rob'][np.isfinite(rr['rob'])]
    st  = 'PASS' if rr['ok'].all() else 'FAIL'
    mr  = fin.min() if len(fin) else float('nan')
    return f'{st}  {rr["ok"].sum():>4}/{T}  {mr:>+7.1f}'

print('=' * 78)
print('  COMPLIANCE COMPARISON  --  ATIC 2026 - Zurich Delivery Mission')
print('=' * 78)
if traj_opt is not None:
    print(f'  {"Rule":<16}  {"Label":<30}  {"NAIVE":^22}  {"OPT":^22}')
else:
    print(f'  {"Rule":<16}  {"Label":<30}  {"NAIVE":^22}')
print('-' * 78)

prev_tier = None
for idx, rule in enumerate(RULES):
    if rule['tier'] != prev_tier:
        prev_tier = rule['tier']
        print(f'\n  {_TIER_LABEL.get(rule["tier"], "Tier " + str(rule["tier"]))}')

    naive_str = _fmt_rule(naive_results, idx)
    line = f'  [{rule["id"]:<14}]  {rule["label"]:<30}  {naive_str}'
    if traj_opt is not None:
        opt_str = _fmt_rule(opt_results, idx)
        line += f'  |  {opt_str}'
    print(line)

print()
print('-' * 78)
naive_pct = 100 * naive_all_ok.mean()
print(f'  Overall compliant:  {naive_all_ok.sum():>4}/{T}  ({naive_pct:.1f}%)   [NAIVE]')
if traj_opt is not None:
    opt_pct = 100 * opt_all_ok.mean()
    print(f'  Overall compliant:  {opt_all_ok.sum():>4}/{T}  ({opt_pct:.1f}%)   [OPT]')
print('=' * 78 + '\n')

fig.show()
