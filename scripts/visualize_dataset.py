#!/usr/bin/env python3
"""Browse recorded teleop datasets and scrub any episode in a rollout viewer.

Auto-detects camera topics (JPEG blobs) and low-dimensional telemetry topics
(joint state, gripper, wrench, ...) from each episode pickle, so it works on any
dataset recorded by bc/data_record, not just box-in-box.

Interactive (default) -- pick a dataset, then any episode, in the browser:

    python scripts/visualize_dataset.py                 # serve ./raw_data
    python scripts/visualize_dataset.py raw_data --port 8010
    python scripts/visualize_dataset.py raw_data/box-in-box   # a single dataset

    then open http://localhost:8000

One-shot export (self-contained HTML, e.g. to share as an artifact):

    python scripts/visualize_dataset.py raw_data/box-in-box -e 7 --export ep7.html
"""
from __future__ import annotations
import argparse, base64, html as htmllib, json, pickle, sys, threading
from functools import lru_cache
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs, quote, unquote

import numpy as np
import cv2

# categorical palette (readable on dark + light); reused per chart
PALETTE = ["#4dd0c4", "#7aa2f7", "#e0af68", "#c69cf7", "#f7768e", "#9ece6a",
           "#ff9e64", "#2ac3de"]


def is_jpeg_list(v):
    return (isinstance(v, list) and v and isinstance(v[0], np.ndarray)
            and v[0].dtype == np.uint8 and v[0].ndim == 1
            and v[0].size > 8 and v[0][0] == 255 and v[0][1] == 216)


def is_lowdim_list(v):
    return (isinstance(v, list) and v and isinstance(v[0], np.ndarray)
            and v[0].ndim == 1 and v[0].dtype != np.uint8 and v[0].size <= 64)


def label_dims(topic, dim):
    t = topic.lower()
    if "wrench" in t and dim == 6:
        return "N · N·m", None, ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    if "gripper" in t:
        names = (["pos", "current"] + [f"d{i}" for i in range(dim)])[:dim]
        return "0–255", (0, 255), names
    if ("state" in t or "joint" in t) and dim == 6:
        return "rad", None, ["pan", "lift", "elbow", "wrist1", "wrist2", "wrist3"]
    return "", None, [f"d{i}" for i in range(dim)]


def encode_frame(blob, width, quality):
    img = cv2.imdecode(blob, cv2.IMREAD_COLOR)          # BGR array of the stored frame
    h = int(round(img.shape[0] * width / img.shape[1]))
    img = cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA)
    # The recorder stored rgb8 through cv2.imencode (which expects BGR), swapping
    # R/B in every saved JPEG. A single channel reversal here makes imencode emit a
    # JPEG the browser displays in true color.
    ok, buf = cv2.imencode(".jpg", img[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode("ascii")


def nearest(src_ts, dst_ts):
    src = np.asarray(src_ts, dtype=np.int64)
    return [int(np.argmin(np.abs(src - t))) for t in dst_ts]


def resolve_episode(path: Path, episode: int) -> Path:
    if path.is_file():
        return path
    eps = sorted(path.glob("ep_*.pkl"))
    if not eps:
        sys.exit(f"no ep_*.pkl found in {path}")
    if episode >= len(eps):
        sys.exit(f"episode {episode} out of range (dataset has {len(eps)})")
    return eps[episode]


def build_payload(ep_path: Path, width: int, quality: int) -> dict:
    raw = pickle.load(open(ep_path, "rb"))
    data, ts = raw["data"], raw.get("timestamps", {})

    cam_topics = [k for k, v in data.items() if is_jpeg_list(v)]
    low_topics = [k for k, v in data.items() if is_lowdim_list(v)]
    if not cam_topics:
        sys.exit("no camera (JPEG) topics found in episode")

    master = max(cam_topics, key=lambda k: len(data[k]))
    master_ts = np.asarray(ts.get(master, np.arange(len(data[master]))), dtype=np.int64)
    n = len(master_ts)
    t_s = (master_ts - master_ts[0]) / 1e9

    def short(topic):
        return topic.strip("/").split("/")[-1]

    def cam_name(topic):
        # /realsense/left/im -> "left"; drop trailing generic image tokens
        parts = [p for p in topic.strip("/").split("/")]
        generic = {"im", "image", "images", "color", "rgb", "compressed"}
        while len(parts) > 1 and parts[-1].lower() in generic:
            parts.pop()
        return parts[-1]

    cameras = []
    for topic in sorted(cam_topics):
        idx = range(n) if topic == master else nearest(
            ts.get(topic, np.arange(len(data[topic]))), master_ts)
        frames = [encode_frame(data[topic][i], width, quality) for i in idx]
        h, w = cv2.imdecode(data[topic][0], cv2.IMREAD_COLOR).shape[:2]
        cameras.append({"name": cam_name(topic), "tag": f"{w}×{h}", "frames": frames})

    charts = []
    for ci, topic in enumerate(sorted(low_topics)):
        arr = np.array([r for r in data[topic]])
        dim = arr.shape[1]
        idx = nearest(ts.get(topic, np.arange(len(arr))), master_ts)
        samp = arr[idx]
        unit, fixed, names = label_dims(topic, dim)
        dims = [{"name": names[d], "color": PALETTE[d % len(PALETTE)],
                 "values": [round(float(x), 4) for x in samp[:, d]]}
                for d in range(dim)]
        charts.append({"label": short(topic), "unit": unit,
                       "fixed": list(fixed) if fixed else None, "dims": dims})

    return {
        "meta": {"dataset": ep_path.parent.name, "episode": ep_path.stem,
                 "frames": int(n), "duration": round(float(t_s[-1]), 2),
                 "fps": round(float((n - 1) / t_s[-1]), 1) if t_s[-1] > 0 else 0,
                 "source": str(ep_path)},
        "time": [round(float(x), 3) for x in t_s],
        "cameras": cameras, "charts": charts,
    }


def render_viewer(payload: dict, nav: str = "") -> str:
    return (TEMPLATE
            .replace("__DATA__", json.dumps(payload, separators=(",", ":")))
            .replace("__NAV__", nav))


# --------------------------------------------------------------- dataset discovery
def episodes_in(d: Path):
    return sorted(d.glob("ep_*.pkl"))


def find_datasets(root: Path):
    """Return {name: [episode paths]}. `root` may be a single dataset (contains
    ep_*.pkl) or a parent holding several dataset subdirs."""
    if episodes_in(root):
        return {root.name: episodes_in(root)}
    out = {}
    for sub in sorted(p for p in root.iterdir() if p.is_dir()):
        eps = episodes_in(sub)
        if eps:
            out[sub.name] = eps
    return out


# --------------------------------------------------------------- browser server
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>__TITLE__</title>
<style>
  :root{--bg:#0d1014;--panel:#151a21;--panel2:#1b222b;--line:#28313c;--ink:#e8eef5;
    --muted:#8797a8;--faint:#5d6b7a;--accent:#4dd0c4;
    --mono:ui-monospace,"SF Mono",Menlo,Consolas,monospace;
    --sans:system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;}
  @media(prefers-color-scheme:light){:root{--bg:#eaeef2;--panel:#fff;--panel2:#f3f6f9;--line:#dae1e9;
    --ink:#16202b;--muted:#5a6a7a;--faint:#8695a4;--accent:#0c9186;}}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);line-height:1.5}
  .wrap{max-width:1000px;margin:0 auto;padding:34px 20px 64px}
  .eyebrow{font-family:var(--mono);font-size:12px;letter-spacing:.14em;text-transform:uppercase;color:var(--accent)}
  h1{font-size:clamp(22px,4vw,32px);font-weight:650;margin:.15em 0 .5em;letter-spacing:-.01em}
  a{color:inherit}
  .crumb{font-family:var(--mono);font-size:12.5px;color:var(--muted);margin-bottom:8px}
  .crumb a{color:var(--accent);text-decoration:none}
  .grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fill,minmax(200px,1fr))}
  .card{display:block;background:var(--panel);border:1px solid var(--line);border-radius:12px;
    padding:16px 18px;text-decoration:none;transition:border-color .15s,transform .1s}
  .card:hover{border-color:var(--accent);transform:translateY(-1px)}
  .card .n{font-weight:600;font-size:15px}
  .card .s{font-family:var(--mono);font-size:12px;color:var(--muted);margin-top:4px}
  .eps{display:grid;gap:8px;grid-template-columns:repeat(auto-fill,minmax(92px,1fr))}
  .ep{font-family:var(--mono);font-size:13px;text-align:center;background:var(--panel);
    border:1px solid var(--line);border-radius:9px;padding:12px 6px;text-decoration:none;
    font-variant-numeric:tabular-nums;transition:border-color .15s}
  .ep:hover{border-color:var(--accent)}
  .empty{font-family:var(--mono);color:var(--muted);font-size:13px}
</style></head><body><div class="wrap">__BODY__</div></body></html>"""


def page(title, body):
    return PAGE.replace("__TITLE__", title).replace("__BODY__", body)


def index_body(datasets):
    cards = "".join(
        f'<a class="card" href="/dataset?name={quote(name)}">'
        f'<div class="n">{htmllib.escape(name)}</div>'
        f'<div class="s">{len(eps)} episodes</div></a>'
        for name, eps in datasets.items())
    if not cards:
        cards = '<div class="empty">no datasets (dirs with ep_*.pkl) found under the root</div>'
    return (f'<div class="eyebrow">teleop datasets</div><h1>Rollout viewer</h1>'
            f'<div class="grid">{cards}</div>')


def dataset_body(name, eps):
    items = "".join(
        f'<a class="ep" href="/view?name={quote(name)}&ep={i}">ep {i:03d}</a>'
        for i in range(len(eps)))
    return (f'<div class="crumb"><a href="/">datasets</a> / {htmllib.escape(name)}</div>'
            f'<h1>{htmllib.escape(name)}</h1>'
            f'<div class="s" style="font-family:var(--mono);color:var(--muted);font-size:12.5px;'
            f'margin-bottom:14px">{len(eps)} episodes — pick one to scrub</div>'
            f'<div class="eps">{items}</div>')


def nav_html(name, ep, n_eps):
    opts = "".join(f'<option value="{i}"{" selected" if i == ep else ""}>ep {i:03d}</option>'
                   for i in range(n_eps))
    q = quote(name)
    prev = f'/view?name={q}&ep={max(0, ep-1)}'
    nxt = f'/view?name={q}&ep={min(n_eps-1, ep+1)}'
    return (f'<div class="nav"><a href="/dataset?name={q}">← {htmllib.escape(name)}</a>'
            f'<label>episode <select id="epsel">{opts}</select></label>'
            f'<a class="step" href="{prev}">◀</a><a class="step" href="{nxt}">▶</a>'
            f'<script>document.getElementById("epsel").onchange='
            f'e=>location.href="/view?name={q}&ep="+e.target.value;</script></div>')


def make_server(root: Path, width: int, quality: int):
    datasets = find_datasets(root)

    @lru_cache(maxsize=16)
    def viewer_for(name, ep):
        eps = datasets[name]
        payload = build_payload(eps[ep], width, quality)
        return render_viewer(payload, nav_html(name, ep, len(eps)))

    class Handler(BaseHTTPRequestHandler):
        def _send(self, body, code=200, ctype="text/html; charset=utf-8"):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            try:
                if u.path == "/":
                    self._send(page("Rollout viewer", index_body(datasets)))
                elif u.path == "/dataset":
                    name = unquote(qs.get("name", [""])[0])
                    if name not in datasets:
                        return self._send(page("not found", '<div class="empty">no such dataset</div>'), 404)
                    self._send(page(name, dataset_body(name, datasets[name])))
                elif u.path == "/view":
                    name = unquote(qs.get("name", [""])[0])
                    ep = int(qs.get("ep", ["0"])[0])
                    if name not in datasets or not (0 <= ep < len(datasets[name])):
                        return self._send(page("not found", '<div class="empty">no such episode</div>'), 404)
                    self._send(viewer_for(name, ep))
                else:
                    self._send(page("not found", '<div class="empty">404</div>'), 404)
            except Exception as e:  # keep the server alive, surface the error in-page
                self._send(page("error", f'<div class="empty">error: {htmllib.escape(str(e))}</div>'), 500)

    return datasets, Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path, nargs="?", default=Path("raw_data"),
                    help="dataset dir, a parent of datasets, or a single ep_*.pkl (default: raw_data)")
    ap.add_argument("-e", "--episode", type=int, default=0, help="episode index (export mode)")
    ap.add_argument("--export", type=Path, metavar="FILE.html",
                    help="write ONE episode to a self-contained HTML and exit (no server)")
    ap.add_argument("--port", type=int, default=8000, help="server port (default 8000)")
    ap.add_argument("-w", "--width", type=int, default=320, help="frame width px (default 320)")
    ap.add_argument("-q", "--quality", type=int, default=60, help="JPEG quality (default 60)")
    args = ap.parse_args()

    if args.export or args.path.is_file():
        ep = resolve_episode(args.path, args.episode)
        print(f"reading {ep}")
        payload = build_payload(ep, args.width, args.quality)
        m = payload["meta"]
        print(f"  {len(payload['cameras'])} cameras, {len(payload['charts'])} telemetry topics, "
              f"{m['frames']} frames, {m['duration']}s (~{m['fps']} Hz)")
        out = args.export or ep.with_suffix(".html")
        out.write_text(render_viewer(payload))
        print(f"wrote {out}  -> open in a browser")
        return

    datasets, Handler = make_server(args.path, args.width, args.quality)
    print(f"found {len(datasets)} dataset(s) under {args.path}: "
          f"{', '.join(datasets) if datasets else '(none)'}")
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"serving on http://localhost:{args.port}  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


TEMPLATE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleop rollout viewer</title>
<style>
  :root{
    --bg:#0d1014;--panel:#151a21;--panel2:#1b222b;--line:#28313c;
    --ink:#e8eef5;--muted:#8797a8;--faint:#5d6b7a;--accent:#4dd0c4;--grid:#212a34;
    --shadow:0 8px 24px rgba(0,0,0,.35);
    --mono:ui-monospace,"SF Mono",Menlo,Consolas,monospace;
    --sans:system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;
  }
  @media(prefers-color-scheme:light){:root{
    --bg:#eaeef2;--panel:#fff;--panel2:#f3f6f9;--line:#dae1e9;
    --ink:#16202b;--muted:#5a6a7a;--faint:#8695a4;--accent:#0c9186;--grid:#e6ecf1;
    --shadow:0 10px 26px rgba(16,32,48,.10);}}
  :root[data-theme="dark"]{--bg:#0d1014;--panel:#151a21;--panel2:#1b222b;--line:#28313c;
    --ink:#e8eef5;--muted:#8797a8;--faint:#5d6b7a;--accent:#4dd0c4;--grid:#212a34;}
  :root[data-theme="light"]{--bg:#eaeef2;--panel:#fff;--panel2:#f3f6f9;--line:#dae1e9;
    --ink:#16202b;--muted:#5a6a7a;--faint:#8695a4;--accent:#0c9186;--grid:#e6ecf1;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);line-height:1.5;
    -webkit-font-smoothing:antialiased}
  .wrap{max-width:1120px;margin:0 auto;padding:28px 20px 64px}
  header{display:flex;flex-wrap:wrap;align-items:baseline;gap:10px 18px;margin-bottom:20px}
  .eyebrow{font-family:var(--mono);font-size:12px;letter-spacing:.14em;text-transform:uppercase;color:var(--accent)}
  h1{font-size:clamp(20px,3.4vw,30px);font-weight:650;margin:0;letter-spacing:-.01em;text-wrap:balance}
  h1 .num{color:var(--muted);font-weight:450}
  .meta{display:flex;flex-wrap:wrap;gap:8px;margin-left:auto}
  .chip{font-family:var(--mono);font-size:11.5px;color:var(--muted);background:var(--panel2);
    border:1px solid var(--line);border-radius:999px;padding:3px 10px;white-space:nowrap}
  .chip b{color:var(--ink);font-weight:600;font-variant-numeric:tabular-nums}
  .nav{display:flex;flex-wrap:wrap;align-items:center;gap:10px;margin:0 0 18px;
    font-family:var(--mono);font-size:12.5px}
  .nav a,.nav select{font-family:var(--mono);font-size:12.5px;color:var(--ink);background:var(--panel2);
    border:1px solid var(--line);border-radius:8px;padding:6px 10px;text-decoration:none;cursor:pointer}
  .nav a:hover{border-color:var(--accent)}
  .nav .step{width:34px;text-align:center;display:inline-block}
  .nav label{color:var(--muted);display:flex;align-items:center;gap:7px}
  .videos{display:grid;gap:14px}
  .cam{background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden;box-shadow:var(--shadow)}
  .cam .bar{display:flex;align-items:center;gap:8px;padding:9px 12px;border-bottom:1px solid var(--line)}
  .dot{width:7px;height:7px;border-radius:50%;background:var(--accent);box-shadow:0 0 8px var(--accent)}
  .cam .name{font-family:var(--mono);font-size:12px}
  .cam .tag{font-family:var(--mono);font-size:11px;color:var(--faint);margin-left:auto}
  .cam img{display:block;width:100%;height:auto;background:#000}
  .transport{display:flex;align-items:center;gap:14px;margin:16px 0 22px;padding:12px 14px;
    background:var(--panel);border:1px solid var(--line);border-radius:12px;box-shadow:var(--shadow)}
  button.play{flex:none;width:42px;height:42px;border-radius:50%;border:1px solid var(--line);
    background:var(--panel2);color:var(--ink);cursor:pointer;display:grid;place-items:center;transition:border-color .15s}
  button.play:hover{border-color:var(--accent)}
  button.play:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
  .scrub{flex:1;display:flex;flex-direction:column;gap:5px}
  input[type=range]{width:100%;accent-color:var(--accent);cursor:pointer;margin:0}
  .clock{display:flex;justify-content:space-between;font-family:var(--mono);font-size:11.5px;
    color:var(--muted);font-variant-numeric:tabular-nums}
  .clock b{color:var(--ink)}
  .speed{flex:none;font-family:var(--mono);font-size:12px;background:var(--panel2);color:var(--ink);
    border:1px solid var(--line);border-radius:8px;padding:7px 8px}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px 16px 8px;
    box-shadow:var(--shadow);margin-bottom:16px}
  .panel h2{font-size:12px;font-family:var(--mono);letter-spacing:.06em;text-transform:uppercase;
    color:var(--muted);font-weight:600;margin:0 0 4px}
  .panel h2 span{color:var(--faint);text-transform:none;letter-spacing:0;margin-left:8px}
  .legend{display:flex;flex-wrap:wrap;gap:7px;margin:6px 0 8px}
  .lg{font-family:var(--mono);font-size:11.5px;background:var(--panel2);border:1px solid var(--line);
    border-radius:7px;padding:4px 8px;display:flex;gap:6px;align-items:center}
  .lg .sw{width:9px;height:9px;border-radius:2px}
  .lg .v{color:var(--ink);font-variant-numeric:tabular-nums;min-width:52px;text-align:right}
  .lg .k{color:var(--muted)}
  canvas{display:block;width:100%}
  .foot{font-family:var(--mono);font-size:11.5px;color:var(--faint);margin-top:26px;
    border-top:1px solid var(--line);padding-top:14px;word-break:break-all}
</style></head><body>
<div class="wrap">
  <header>
    <div><div class="eyebrow">teleop · demonstration</div>
      <h1 id="title"></h1></div>
    <div class="meta" id="meta"></div>
  </header>
  __NAV__
  <div class="videos" id="videos"></div>
  <div class="transport">
    <button class="play" id="play" aria-label="Play"></button>
    <div class="scrub"><input type="range" id="seek" min="0" value="0" step="1">
      <div class="clock"><span>frame <b id="fIdx">0</b> / <span id="fMax">0</span></span>
        <span><b id="tCur">0.00</b> / <span id="tTot">0.00</span> s</span></div></div>
    <select class="speed" id="speed" aria-label="Speed">
      <option value="0.5">0.5×</option><option value="1" selected>1×</option>
      <option value="2">2×</option><option value="4">4×</option></select>
  </div>
  <div id="charts"></div>
  <div class="foot" id="foot"></div>
</div>
<script>
const D=__DATA__;
const css=k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim();
const N=D.meta.frames,T=D.time,TOT=D.meta.duration;

document.getElementById("title").innerHTML=
  `${D.meta.dataset} <span class="num">/ ${D.meta.episode}</span>`;
document.getElementById("meta").innerHTML=
  `<span class="chip"><b>${N}</b> frames</span><span class="chip"><b>${TOT}</b> s</span>`+
  `<span class="chip">~<b>${D.meta.fps}</b> Hz</span>`+
  `<span class="chip">${D.cameras.length} cam${D.cameras.length>1?"s":""}</span>`;
document.getElementById("fMax").textContent=N-1;
document.getElementById("tTot").textContent=TOT.toFixed(2);
document.getElementById("seek").max=N-1;
document.getElementById("foot").textContent=D.meta.source;

// cameras
const vids=document.getElementById("videos");
vids.style.gridTemplateColumns=`repeat(${Math.min(D.cameras.length,2)},1fr)`;
const camEls=D.cameras.map(c=>{
  const fig=document.createElement("figure");fig.className="cam";fig.style.margin="0";
  fig.innerHTML=`<div class="bar"><span class="dot"></span><span class="name">${c.name}</span>`+
    `<span class="tag">${c.tag}</span></div><img alt="${c.name}">`;
  vids.appendChild(fig);
  return {img:fig.querySelector("img"),imgs:c.frames.map(b=>{const i=new Image();i.src="data:image/jpeg;base64,"+b;return i;})};
});

// charts
const DPR=Math.min(2,window.devicePixelRatio||1);
const PAD={l:46,r:12,t:10,b:20};
function range(dims,fixed){ if(fixed)return fixed;
  let mn=Infinity,mx=-Infinity;for(const d of dims)for(const v of d.values){if(v<mn)mn=v;if(v>mx)mx=v;}
  if(mn===mx){mn-=1;mx+=1;}const p=(mx-mn)*0.08;return [mn-p,mx+p];}
const chartsRoot=document.getElementById("charts");
const charts=D.charts.map(ch=>{
  const rng=range(ch.dims,ch.fixed);
  const p=document.createElement("div");p.className="panel";
  const legend=ch.dims.map((d,i)=>
    `<span class="lg"><span class="sw" style="background:${d.color}"></span>`+
    `<span class="k">${d.name}</span><span class="v" data-d="${i}">·</span></span>`).join("");
  p.innerHTML=`<h2>${ch.label}<span>${ch.unit||""}</span></h2>`+
    `<div class="legend">${legend}</div><canvas></canvas>`;
  chartsRoot.appendChild(p);
  return {ch,rng,canvas:p.querySelector("canvas"),vals:p.querySelectorAll(".v")};
});
function fit(cv,h){const w=cv.parentElement.clientWidth-32;cv.width=w*DPR;cv.height=h*DPR;
  cv.style.height=h+"px";const c=cv.getContext("2d");c.setTransform(DPR,0,0,DPR,0,0);return[c,w,h];}
function xAt(i,w){return PAD.l+(w-PAD.l-PAD.r)*(N<2?0:i/(N-1));}
function yAt(v,h,lo,hi){return PAD.t+(h-PAD.t-PAD.b)*(1-(v-lo)/(hi-lo));}
function drawChart(o){
  const[c,w,h]=fit(o.canvas,150);const[lo,hi]=o.rng;
  c.clearRect(0,0,w,h);c.strokeStyle=css("--grid");c.fillStyle=css("--faint");
  c.lineWidth=1;c.font="10px "+css("--mono");c.textAlign="right";c.textBaseline="middle";
  for(let i=0;i<=4;i++){const val=lo+(hi-lo)*i/4,y=PAD.t+(h-PAD.t-PAD.b)*(1-i/4);
    c.beginPath();c.moveTo(PAD.l,y);c.lineTo(w-PAD.r,y);c.stroke();
    c.fillText(Math.abs(val)>=50?val.toFixed(0):val.toFixed(1),PAD.l-6,y);}
  for(const d of o.ch.dims){c.strokeStyle=d.color;c.lineWidth=1.6;c.lineJoin="round";c.beginPath();
    for(let i=0;i<N;i++){const x=xAt(i,w),y=yAt(d.values[i],h,lo,hi);i?c.lineTo(x,y):c.moveTo(x,y);}c.stroke();}
  const x=xAt(cur,w);c.save();c.strokeStyle=css("--accent");c.globalAlpha=.9;c.lineWidth=1;
  c.beginPath();c.moveTo(x,PAD.t);c.lineTo(x,h-PAD.b);c.stroke();c.restore();
}
function renderCharts(){charts.forEach(drawChart);}

let cur=0,playing=false,last=0,acc=0;
const seek=document.getElementById("seek"),playBtn=document.getElementById("play");
const PL='<svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor"><path d="M4 3l9 5-9 5z"/></svg>';
const PA='<svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor"><rect x="4" y="3" width="3" height="10"/><rect x="9" y="3" width="3" height="10"/></svg>';
playBtn.innerHTML=PL;
function setFrame(i){
  cur=Math.max(0,Math.min(N-1,i|0));
  camEls.forEach(c=>c.img.src=c.imgs[cur].src);
  seek.value=cur;
  document.getElementById("fIdx").textContent=cur;
  document.getElementById("tCur").textContent=T[cur].toFixed(2);
  charts.forEach(o=>o.vals.forEach((el,d)=>{
    const v=o.ch.dims[d].values[cur];el.textContent=Math.abs(v)>=50?v.toFixed(0):v.toFixed(3);}));
  renderCharts();
}
function loop(ts){ if(!playing)return; if(!last)last=ts;
  const spd=parseFloat(document.getElementById("speed").value);
  acc+=(ts-last)/1000*spd;last=ts;
  const dt=(T[Math.min(cur+1,N-1)]-T[cur])||(1/(D.meta.fps||15));
  if(acc>=dt){let nx=cur+1;if(nx>=N)nx=0;setFrame(nx);acc=0;}
  requestAnimationFrame(loop);}
function play(){if(playing)return;playing=true;last=0;acc=0;playBtn.innerHTML=PA;
  playBtn.setAttribute("aria-label","Pause");requestAnimationFrame(loop);}
function pause(){playing=false;playBtn.innerHTML=PL;playBtn.setAttribute("aria-label","Play");}
playBtn.onclick=()=>playing?pause():play();
seek.oninput=()=>{pause();setFrame(+seek.value);};
document.getElementById("speed").onchange=()=>{last=0;acc=0;};
addEventListener("keydown",e=>{
  if(e.key===" "){e.preventDefault();playing?pause():play();}
  else if(e.key==="ArrowRight"){pause();setFrame(cur+1);}
  else if(e.key==="ArrowLeft"){pause();setFrame(cur-1);}});
let rt;addEventListener("resize",()=>{clearTimeout(rt);rt=setTimeout(renderCharts,120);});
new MutationObserver(renderCharts).observe(document.documentElement,{attributes:true,attributeFilter:["data-theme"]});
setFrame(0);
</script></body></html>"""


if __name__ == "__main__":
    main()
