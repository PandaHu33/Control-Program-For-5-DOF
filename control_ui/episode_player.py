#!/usr/bin/env python3
"""Interactive local replay for a recorded Control-Program episode.

The player uses only Python's standard library.  It serves the episode through
localhost so browsers can seek both MP4 files with HTTP range requests, then
aligns video frames and telemetry on the timestamps captured by the recording
service.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import mimetypes
import re
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse


CONTROL_UI_DIR = Path(__file__).resolve().parent
DEFAULT_RECORDINGS_DIR = CONTROL_UI_DIR / "recordings"
CHANNEL_NAMES = ["thumb_yaw", "thumb_pitch", "index", "middle", "ring", "pinky"]


class EpisodeError(RuntimeError):
    pass


def _number(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number == number and abs(number) != float("inf") else None


def _integer(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _json_value(value: str | None, default: Any) -> Any:
    if not value:
        return default
    try:
        return json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return default


def _vector(row: dict[str, str], prefix: str, count: int) -> list[float | None]:
    return [_number(row.get(f"{prefix}_{index}")) for index in range(1, count + 1)]


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def find_latest_episode(recordings_dir: Path = DEFAULT_RECORDINGS_DIR) -> Path:
    candidates: list[tuple[int, Path]] = []
    if recordings_dir.is_dir():
        for manifest_path in recordings_dir.glob("*/*/manifest.json"):
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
                timestamp = int(manifest.get("started_at_ns", 0))
            except (OSError, ValueError, json.JSONDecodeError):
                timestamp = 0
            candidates.append((timestamp, manifest_path.parent))
    if not candidates:
        raise EpisodeError(f"没有找到 Episode：{recordings_dir}")
    return max(candidates, key=lambda item: (item[0], str(item[1])))[1]


def resolve_episode(path_text: str | None) -> Path:
    episode = Path(path_text).expanduser() if path_text else find_latest_episode()
    if not episode.is_absolute():
        episode = (Path.cwd() / episode).resolve()
    else:
        episode = episode.resolve()
    if not (episode / "manifest.json").is_file():
        raise EpisodeError(f"目录中没有 manifest.json：{episode}")
    return episode


def _frame_map(path: Path, started_ns: int) -> list[list[float]]:
    result: list[list[float]] = []
    for row in _read_csv(path):
        capture_ns = _integer(row.get("capture_utc_ns"))
        pts_ns = _integer(row.get("encoded_pts_ns"))
        if capture_ns is None or pts_ns is None:
            continue
        result.append([round((capture_ns - started_ns) / 1e9, 6), round(pts_ns / 1e9, 6)])
    return result


def load_episode(episode: Path) -> dict[str, Any]:
    try:
        manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EpisodeError(f"无法读取 manifest.json：{exc}") from exc

    started_ns = int(manifest.get("started_at_ns") or 0)
    if started_ns <= 0:
        raise EpisodeError("manifest.json 缺少有效的 started_at_ns")

    alignment_file = str(manifest.get("alignment_file") or "").strip()
    alignment_candidates = [episode / alignment_file] if alignment_file else []
    alignment_candidates.extend([episode / "aligned_30hz.csv", episode / "aligned_20hz.csv"])
    alignment_path = next((path for path in alignment_candidates if path.is_file()), None)
    if alignment_path is None:
        alignment_path = next(iter(sorted(episode.glob("aligned_*hz.csv"))), episode / "aligned_30hz.csv")
    aligned_rows = _read_csv(alignment_path)
    if not aligned_rows:
        raise EpisodeError(f"{alignment_path.name} 不存在或没有数据")

    monitor_rows = _read_csv(episode / "episode_records.csv")
    monitor_times: list[float] = []
    monitors: list[dict[str, Any]] = []
    for row in monitor_rows:
        timestamp_ns = _integer(row.get("timestamp_ns"))
        if timestamp_ns is None:
            continue
        monitor_times.append((timestamp_ns - started_ns) / 1e9)
        residual = _json_value(row.get("current_residual_json"), [None] * 6)
        if not isinstance(residual, list):
            residual = [None] * 6
        residual = [(_number(value)) for value in (residual + [None] * 6)[:6]]
        monitors.append(
            {
                "state": row.get("state") or "UNKNOWN",
                "valid": row.get("valid") == "1",
                "closure": _number(row.get("closure_score")),
                "residual": residual,
                "loaded": _json_value(row.get("loaded_fingers_json"), []),
                "reason": row.get("reason") or "",
                "invalidReason": row.get("invalid_reason") or "",
                "prompt": row.get("prompt_event") or "",
            }
        )

    samples: list[dict[str, Any]] = []
    for row in aligned_rows:
        session_time_ns = _integer(row.get("session_time_ns"))
        if session_time_ns is None:
            continue
        time_sec = session_time_ns / 1e9
        monitor: dict[str, Any] | None = None
        if monitor_times:
            monitor_index = bisect.bisect_right(monitor_times, time_sec) - 1
            if monitor_index >= 0 and time_sec - monitor_times[monitor_index] <= 0.25:
                monitor = monitors[monitor_index]
        samples.append(
            {
                "t": round(time_sec, 6),
                "armValid": row.get("arm_valid") == "1",
                "handValid": row.get("hand_valid") == "1",
                "armMode": row.get("arm_control_mode") or "",
                "handMode": row.get("hand_control_mode") or "",
                "armQ": _vector(row, "arm_actual_q_rad", 5),
                "armTarget": _vector(row, "arm_target_q_rad", 5),
                "armCurrent": _vector(row, "arm_actual_current_ma", 5),
                "handPosition": _vector(row, "hand_actual_position_units", 6),
                "handTarget": _vector(row, "hand_target_position_units", 6),
                "handCurrent": _vector(row, "hand_filtered_current_ma", 6),
                "monitor": monitor,
            }
        )

    duration = max(float(manifest.get("duration_sec") or 0.0), samples[-1]["t"] if samples else 0.0)
    videos = {}
    frames = {}
    for side in ("left", "right"):
        video_path = episode / f"{side}.mp4"
        frames[side] = _frame_map(episode / f"{side}_frames.csv", started_ns)
        videos[side] = {
            "available": video_path.is_file(),
            "size": video_path.stat().st_size if video_path.is_file() else 0,
            "frameCount": len(frames[side]),
        }

    return {
        "meta": {
            "sessionId": manifest.get("session_id") or episode.name,
            "directory": str(episode),
            "date": manifest.get("date") or "",
            "method": manifest.get("method") or "",
            "state": manifest.get("state") or "",
            "complete": bool(manifest.get("complete")),
            "duration": round(duration, 6),
            "alignmentHz": manifest.get("alignment_hz") or 30,
            "counts": manifest.get("counts") or {},
            "errors": manifest.get("errors") or [],
            "videos": videos,
        },
        "channelNames": CHANNEL_NAMES,
        "samples": samples,
        "frames": frames,
    }


PLAYER_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Episode Replay</title>
<style>
:root{color-scheme:dark;--bg:#0d1117;--panel:#161b22;--line:#30363d;--text:#e6edf3;--muted:#8b949e;--accent:#58a6ff;--bad:#f85149;--ok:#3fb950}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1500px;margin:auto;padding:16px}.top{display:flex;gap:12px;align-items:flex-end;justify-content:space-between;flex-wrap:wrap;margin-bottom:12px}
h1{font-size:20px;margin:0 0 3px}.muted{color:var(--muted)}.videos{display:grid;grid-template-columns:1fr 1fr;gap:10px}.video-box{position:relative;background:#000;border:1px solid var(--line);border-radius:8px;overflow:hidden;aspect-ratio:4/3}.video-box video{width:100%;height:100%;object-fit:contain}.video-label{position:absolute;left:8px;top:7px;background:#000a;padding:3px 7px;border-radius:4px}.video-missing{display:none;position:absolute;inset:0;align-items:center;justify-content:center;background:#000b;color:#fff}.controls{margin:12px 0;padding:12px;background:var(--panel);border:1px solid var(--line);border-radius:8px}.control-row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.control-row button,.control-row select{background:#21262d;color:var(--text);border:1px solid var(--line);border-radius:6px;padding:6px 10px}.control-row button{cursor:pointer}.time{font-variant-numeric:tabular-nums;min-width:112px}.timeline{width:100%;margin-top:10px;accent-color:var(--accent)}
.monitor{display:grid;grid-template-columns:auto 1fr;gap:6px 12px;margin-top:10px;padding-top:10px;border-top:1px solid var(--line)}.badge{display:inline-block;padding:2px 7px;border-radius:999px;background:#30363d}.badge.valid{background:#173b23;color:#7ee787}.badge.invalid{background:#4b1f24;color:#ff7b72}.reason{word-break:break-word;color:var(--muted)}
.charts{display:grid;gap:10px}.plot{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:8px}.plot-head{display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap;margin-bottom:3px}.legend{color:var(--muted);font-size:12px}.plot canvas{width:100%;height:190px;display:block}.current{padding:8px 2px;color:var(--muted);font-variant-numeric:tabular-nums;white-space:normal}
@media(max-width:760px){main{padding:10px}.videos{grid-template-columns:1fr}.plot canvas{height:165px}.monitor{grid-template-columns:1fr}}
</style>
</head>
<body>
<main>
  <div class="top"><div><h1 id="title">Episode</h1><div id="meta" class="muted"></div></div><div class="control-row"><label>视窗 <select id="window"><option value="0">全程</option><option value="10">10 秒</option><option value="30">30 秒</option></select></label></div></div>
  <section class="videos" aria-label="双相机视频">
    <div class="video-box"><video id="leftVideo" preload="metadata" muted playsinline></video><span class="video-label">左相机</span><div id="leftMissing" class="video-missing">该时刻无左相机帧</div></div>
    <div class="video-box"><video id="rightVideo" preload="metadata" muted playsinline></video><span class="video-label">右相机</span><div id="rightMissing" class="video-missing">该时刻无右相机帧</div></div>
  </section>
  <section class="controls">
    <div class="control-row"><button id="play" type="button">播放</button><button id="stepBack" type="button">−1 帧</button><button id="stepForward" type="button">+1 帧</button><label>速度 <select id="speed"><option value="0.25">0.25×</option><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select></label><span id="time" class="time">00:00.000</span><span class="muted">空格播放；←/→逐帧；Shift+←/→跳 1 秒</span></div>
    <input id="timeline" class="timeline" type="range" min="0" max="1" step="0.001" value="0" aria-label="Episode 进度">
    <div class="monitor"><div><span id="state" class="badge">UNKNOWN</span> <span id="valid" class="badge invalid">无效</span></div><div id="reason" class="reason"></div><div class="muted">受载手指</div><div id="loaded">—</div></div>
  </section>
  <section class="charts">
    <div class="plot"><div class="plot-head"><strong>机械臂关节位置</strong><span class="legend">实线：实际值　虚线：目标值　单位 rad</span></div><canvas id="armPlot" role="img" aria-label="机械臂五关节位置曲线"></canvas></div>
    <div class="plot"><div class="plot-head"><strong>WA100 实际/目标位置</strong><span class="legend">六通道，单位 position units</span></div><canvas id="handPlot" role="img" aria-label="WA100 六通道位置曲线"></canvas></div>
    <div class="plot"><div class="plot-head"><strong>WA100 滤波电流</strong><span class="legend">电流仅作为负载代理，单位 mA</span></div><canvas id="currentPlot" role="img" aria-label="WA100 六通道滤波电流曲线"></canvas></div>
  </section>
  <div id="current" class="current"></div>
</main>
<script>
const COLORS=['#58a6ff','#3fb950','#d29922','#bc8cff','#f778ba','#39c5cf'];
let DATA=null,currentTime=0,playing=false,anchorTime=0,anchorWall=0,lastDraw=-1;
const $=id=>document.getElementById(id); const videos={left:$('leftVideo'),right:$('rightVideo')};
function fmtTime(sec){sec=Math.max(0,sec||0);const m=Math.floor(sec/60),s=Math.floor(sec%60),ms=Math.floor((sec%1)*1000);return `${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}.${String(ms).padStart(3,'0')}`}
function lowerBound(values,target,key='t'){let lo=0,hi=values.length;while(lo<hi){const mid=(lo+hi)>>1;if(values[mid][key]<target)lo=mid+1;else hi=mid}return lo}
function sampleAt(t){const rows=DATA.samples;let i=lowerBound(rows,t);if(i>=rows.length)return rows[rows.length-1];if(i>0&&Math.abs(rows[i-1].t-t)<Math.abs(rows[i].t-t))i--;return rows[i]}
function frameMapping(side,t){const rows=DATA.frames[side];if(rows.length<2)return null;const last=rows.length-1;if(t<rows[0][0]-.1||t>rows[last][0]+.1)return null;let lo=0,hi=rows.length;while(lo<hi){const mid=(lo+hi)>>1;if(rows[mid][0]<t)lo=mid+1;else hi=mid}const right=Math.min(Math.max(lo,1),last),left=right-1,a=rows[left],b=rows[right],span=Math.max(b[0]-a[0],1e-6),mix=Math.max(0,Math.min(1,(t-a[0])/span)),mediaTime=a[1]+(b[1]-a[1])*mix;const slopeLeft=Math.max(0,left-30),slopeRight=Math.min(last,right+30),captureSpan=rows[slopeRight][0]-rows[slopeLeft][0],mediaSpan=rows[slopeRight][1]-rows[slopeLeft][1],rate=captureSpan>1e-6?mediaSpan/captureSpan:1;return{time:mediaTime,rate:Math.max(.05,Math.min(2,rate))}}
function syncVideo(side,hard=false){const video=videos[side],missing=$(side+'Missing'),mapping=frameMapping(side,currentTime);if(mapping===null||!DATA.meta.videos[side].available){missing.style.display='flex';if(!video.paused)video.pause();return}missing.style.display='none';const speed=Number($('speed').value),error=mapping.time-video.currentTime,baseRate=Math.max(.0625,Math.min(16,speed*mapping.rate));if(hard||!Number.isFinite(video.currentTime)||Math.abs(error)>.75){try{video.currentTime=mapping.time}catch(_){}}const correction=hard?1:Math.max(.85,Math.min(1.15,1+error*.25)),desiredRate=Math.max(.0625,Math.min(16,baseRate*correction));if(Math.abs(video.playbackRate-desiredRate)>.015)video.playbackRate=desiredRate;if(playing&&video.paused)video.play().catch(()=>{});if(!playing&&!video.paused)video.pause()}
function valuesText(values,digits=2){return values.map(v=>v===null?'—':Number(v).toFixed(digits)).join(' · ')}
function updateReadout(sample){const monitor=sample.monitor;$('state').textContent=monitor?monitor.state:'NO_DATA';$('valid').textContent=monitor&&monitor.valid?'有效':'无效';$('valid').className='badge '+(monitor&&monitor.valid?'valid':'invalid');$('reason').textContent=monitor?(monitor.invalidReason||monitor.reason||monitor.prompt||'—'):'该时刻没有一致性判别记录';$('loaded').textContent=monitor&&monitor.loaded&&monitor.loaded.length?monitor.loaded.join(', '):'—';$('current').textContent=`机械臂 ${sample.armMode||'—'} q: ${valuesText(sample.armQ,3)}　|　WA100 ${sample.handMode||'—'} 位置: ${valuesText(sample.handPosition,0)}　|　滤波电流: ${valuesText(sample.handCurrent,1)} mA`}
function visibleBounds(){const w=Number($('window').value);if(!w)return [0,DATA.meta.duration];let a=Math.max(0,currentTime-w/2),b=Math.min(DATA.meta.duration,a+w);a=Math.max(0,b-w);return[a,b]}
function drawPlot(canvas,actualKey,targetKey,count){const rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(320,rect.width),h=Math.max(140,rect.height);canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);const pad={l:58,r:12,t:20,b:25},pw=w-pad.l-pad.r,ph=h-pad.t-pad.b,[t0,t1]=visibleBounds();const rows=DATA.samples,i0=Math.max(0,lowerBound(rows,t0)-1),i1=Math.min(rows.length,lowerBound(rows,t1)+1);let min=Infinity,max=-Infinity;for(let i=i0;i<i1;i++){for(const key of [actualKey,targetKey].filter(Boolean)){for(const v of rows[i][key])if(v!==null){min=Math.min(min,v);max=Math.max(max,v)}}}if(!Number.isFinite(min)){min=0;max=1}if(max-min<1e-9){min-=1;max+=1}const margin=(max-min)*.08;min-=margin;max+=margin;const x=t=>pad.l+(t-t0)/(t1-t0||1)*pw,y=v=>pad.t+(max-v)/(max-min)*ph;c.strokeStyle='#30363d';c.fillStyle='#8b949e';c.font='11px system-ui';c.lineWidth=1;for(let k=0;k<=4;k++){const yy=pad.t+ph*k/4,val=max-(max-min)*k/4;c.beginPath();c.moveTo(pad.l,yy);c.lineTo(w-pad.r,yy);c.stroke();c.fillText(val.toFixed(Math.abs(max-min)<10?2:0),4,yy+4)}for(let k=0;k<=5;k++){const tt=t0+(t1-t0)*k/5,xx=x(tt);c.beginPath();c.moveTo(xx,pad.t);c.lineTo(xx,pad.t+ph);c.stroke();c.fillText(fmtTime(tt).slice(0,5),Math.max(pad.l,xx-16),h-7)}function line(key,j,dashed,alpha){if(!key)return;c.strokeStyle=COLORS[j%COLORS.length];c.globalAlpha=alpha;c.lineWidth=dashed?1:1.7;c.setLineDash(dashed?[5,4]:[]);c.beginPath();let started=false;for(let i=i0;i<i1;i++){const v=rows[i][key][j];if(v===null){started=false;continue}const xx=x(rows[i].t),yy=y(v);if(!started){c.moveTo(xx,yy);started=true}else c.lineTo(xx,yy)}c.stroke()}for(let j=0;j<count;j++){line(targetKey,j,true,.55);line(actualKey,j,false,1)}c.setLineDash([]);c.globalAlpha=1;const cursor=x(currentTime);c.strokeStyle='#f0f6fc';c.lineWidth=1;c.beginPath();c.moveTo(cursor,pad.t);c.lineTo(cursor,pad.t+ph);c.stroke();for(let j=0;j<count;j++){c.fillStyle=COLORS[j%COLORS.length];c.fillRect(pad.l+j*78,pad.t-15,9,3);c.fillStyle='#c9d1d9';c.fillText(actualKey==='armQ'?`J${j+1}`:DATA.channelNames[j],pad.l+13+j*78,pad.t-10)}}
function drawAll(force=false){if(!force&&Math.abs(currentTime-lastDraw)<1/30)return;lastDraw=currentTime;drawPlot($('armPlot'),'armQ','armTarget',5);drawPlot($('handPlot'),'handPosition','handTarget',6);drawPlot($('currentPlot'),'handCurrent',null,6)}
function setTime(value,hardVideo=false){currentTime=Math.max(0,Math.min(DATA.meta.duration,Number(value)||0));$('timeline').value=String(currentTime);$('time').textContent=`${fmtTime(currentTime)} / ${fmtTime(DATA.meta.duration)}`;const sample=sampleAt(currentTime);updateReadout(sample);syncVideo('left',hardVideo);syncVideo('right',hardVideo);drawAll(hardVideo)}
function setPlaying(next){playing=next&&currentTime<DATA.meta.duration;anchorTime=currentTime;anchorWall=performance.now();$('play').textContent=playing?'暂停':'播放';syncVideo('left',true);syncVideo('right',true)}
function tick(now){if(playing){const speed=Number($('speed').value);const next=anchorTime+(now-anchorWall)/1000*speed;if(next>=DATA.meta.duration){setTime(DATA.meta.duration,true);setPlaying(false)}else setTime(next,false)}requestAnimationFrame(tick)}
async function init(){const response=await fetch('/episode.json');DATA=await response.json();$('title').textContent=DATA.meta.sessionId;$('meta').textContent=`${DATA.meta.method||'未标注方法'} · ${DATA.meta.duration.toFixed(2)} 秒 · ${DATA.samples.length} 个对齐采样点 · ${DATA.meta.directory}`;$('timeline').max=String(DATA.meta.duration);for(const side of ['left','right'])if(DATA.meta.videos[side].available)videos[side].src=`/media/${side}.mp4`;setTime(0,true);$('play').onclick=()=>setPlaying(!playing);$('stepBack').onclick=()=>{setPlaying(false);setTime(currentTime-1/(DATA.meta.alignmentHz||30),true)};$('stepForward').onclick=()=>{setPlaying(false);setTime(currentTime+1/(DATA.meta.alignmentHz||30),true)};$('timeline').oninput=e=>{if(playing){anchorTime=Number(e.target.value);anchorWall=performance.now()}setTime(e.target.value,true)};$('speed').onchange=()=>{if(playing){anchorTime=currentTime;anchorWall=performance.now()}syncVideo('left');syncVideo('right')};$('window').onchange=()=>drawAll(true);window.addEventListener('resize',()=>drawAll(true));document.addEventListener('keydown',e=>{if(e.target.matches('input,select,button'))return;if(e.code==='Space'){e.preventDefault();setPlaying(!playing)}else if(e.key==='ArrowLeft'){e.preventDefault();setPlaying(false);setTime(currentTime-(e.shiftKey?1:1/(DATA.meta.alignmentHz||30)),true)}else if(e.key==='ArrowRight'){e.preventDefault();setPlaying(false);setTime(currentTime+(e.shiftKey?1:1/(DATA.meta.alignmentHz||30)),true)}});requestAnimationFrame(tick)}
init().catch(error=>{document.body.innerHTML=`<main><h1>Episode 加载失败</h1><pre>${String(error)}</pre></main>`});
</script>
</body>
</html>
"""


class EpisodeRequestHandler(BaseHTTPRequestHandler):
    data_bytes = b"{}"
    episode_dir = Path()

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"[episode-player] {self.address_string()} {format_string % args}")

    def _headers(self, status: int, content_type: str, length: int, extra: dict[str, str] | None = None) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()

    def _bytes(self, payload: bytes, content_type: str) -> None:
        self._headers(200, content_type, len(payload))
        self.wfile.write(payload)

    def _video(self, path: Path) -> None:
        if not path.is_file() or path.parent.resolve() != self.episode_dir.resolve():
            self.send_error(404)
            return
        total = path.stat().st_size
        start, end, status = 0, total - 1, 200
        range_header = self.headers.get("Range", "")
        match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header.strip()) if range_header else None
        if match:
            status = 206
            if match.group(1):
                start = int(match.group(1))
                end = min(int(match.group(2)) if match.group(2) else total - 1, total - 1)
            elif match.group(2):
                length = min(int(match.group(2)), total)
                start, end = total - length, total - 1
            if start > end or start >= total:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{total}")
                self.end_headers()
                return
        length = end - start + 1
        extra = {"Accept-Ranges": "bytes"}
        if status == 206:
            extra["Content-Range"] = f"bytes {start}-{end}/{total}"
        self._headers(status, mimetypes.guess_type(path.name)[0] or "video/mp4", length, extra)
        with path.open("rb") as handle:
            handle.seek(start)
            remaining = length
            while remaining:
                chunk = handle.read(min(1024 * 1024, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = unquote(urlparse(self.path).path)
        if path in {"/", "/index.html"}:
            self._bytes(PLAYER_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/episode.json":
            self._bytes(self.data_bytes, "application/json; charset=utf-8")
        elif path in {"/media/left.mp4", "/media/right.mp4"}:
            self._video(self.episode_dir / Path(path).name)
        elif path == "/favicon.ico":
            self._headers(204, "image/x-icon", 0)
        else:
            self.send_error(404)


def inspect_summary(data: dict[str, Any]) -> str:
    meta = data["meta"]
    return json.dumps(
        {
            "session_id": meta["sessionId"],
            "directory": meta["directory"],
            "duration_sec": meta["duration"],
            "aligned_samples": len(data["samples"]),
            "left_frames": len(data["frames"]["left"]),
            "right_frames": len(data["frames"]["right"]),
            "left_video": meta["videos"]["left"]["available"],
            "right_video": meta["videos"]["right"]["available"],
        },
        ensure_ascii=False,
        indent=2,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="回放双相机、机械臂、WA100 和一致性判别 Episode")
    parser.add_argument("episode", nargs="?", help="Episode 目录；省略时自动选择 recordings 下最新一组")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址，默认仅本机")
    parser.add_argument("--port", type=int, default=8765, help="监听端口；0 表示自动选择")
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    parser.add_argument("--inspect", action="store_true", help="只检查并打印 Episode 摘要")
    args = parser.parse_args()

    try:
        episode = resolve_episode(args.episode)
        data = load_episode(episode)
    except EpisodeError as exc:
        parser.error(str(exc))
    if args.inspect:
        print(inspect_summary(data))
        return 0

    EpisodeRequestHandler.episode_dir = episode
    EpisodeRequestHandler.data_bytes = json.dumps(data, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    server = ThreadingHTTPServer((args.host, args.port), EpisodeRequestHandler)
    url = f"http://{args.host}:{server.server_port}/"
    print(f"Episode: {episode}")
    print(f"回放地址: {url}")
    print("按 Ctrl+C 停止。")
    if not args.no_browser:
        threading.Timer(0.35, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nEpisode 回放已停止。")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
