"""Haritadan görev planlayıcı: mission/map/harita.jpg + harita.json -> tek dosyalık HTML sayfası.

  python mission/map_capture.py        # (simülatör açıkken) haritayı çek
  python tools/gorev_planlayici.py     # mission/map/gorev_planlayici.html üretir; çift tıkla aç

Sayfada: arama alanını dikdörtgen çiz, kontrol noktaları ekle, hedef/irtifa/paket/rüzgar seç,
"Görevi kaydet" ile gorev.json indir. Uçuş:  python mission/run_autonomy.py --model ... --gorev gorev.json
Sayfa internet ya da sunucu gerektirmez (görüntü sayfaya gömülü).
"""
import base64
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MAP = ROOT / "mission" / "map"

PAGE = r"""<!doctype html>
<html lang="tr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Görev Planlayıcı</title>
<style>
 :root { --bg:#f5f6f8; --panel:#ffffff; --text:#1d2330; --muted:#5b6475; --line:#d7dbe3; --accent:#d4382c; --ok:#1f8a4c; }
 @media (prefers-color-scheme: dark) { :root { --bg:#14171d; --panel:#1d222b; --text:#e6e9ef; --muted:#9aa3b2; --line:#2e3542; } }
 * { box-sizing:border-box; }
 body { margin:0; font:14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; background:var(--bg); color:var(--text); }
 header { padding:12px 16px; border-bottom:1px solid var(--line); display:flex; gap:12px; align-items:baseline; flex-wrap:wrap; }
 header h1 { font-size:17px; margin:0; } header span { color:var(--muted); }
 main { display:grid; grid-template-columns: minmax(0,1fr) 300px; gap:14px; padding:14px; }
 @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
 .mapbox { position:relative; background:#000; border-radius:8px; overflow:hidden; }
 canvas { display:block; width:100%; height:auto; cursor:crosshair; }
 .panel { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:14px; display:flex; flex-direction:column; gap:12px; }
 .tools { display:flex; gap:6px; flex-wrap:wrap; }
 button { font:inherit; padding:7px 10px; border-radius:6px; border:1px solid var(--line); background:var(--panel); color:var(--text); cursor:pointer; }
 button.on { border-color:var(--accent); color:var(--accent); font-weight:600; }
 button.primary { background:var(--accent); border-color:var(--accent); color:#fff; font-weight:600; }
 label { display:flex; flex-direction:column; gap:4px; color:var(--muted); font-size:13px; }
 input, select { font:inherit; padding:6px 8px; border-radius:6px; border:1px solid var(--line); background:var(--bg); color:var(--text); width:100%; min-width:0; }
 .row > label, .panel { min-width:0; }
 .row { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
 .summary { font-size:13px; color:var(--muted); white-space:pre-line; border-top:1px solid var(--line); padding-top:10px; }
 .warn { color:var(--accent); }
 .pos { position:absolute; left:8px; bottom:8px; background:rgba(0,0,0,.6); color:#fff; padding:3px 7px; border-radius:4px; font-size:12px; }
</style></head><body>
<header><h1>Görev Planlayıcı</h1><span>Haritada arama alanını çiz, kontrol noktaları ekle, ayarları seç ve görevi kaydet.</span></header>
<main>
 <div class="mapbox"><canvas id="c"></canvas><div class="pos" id="pos">—</div></div>
 <div class="panel">
  <div class="tools">
   <button id="tArea" class="on">Alan çiz</button><button id="tPt">Nokta ekle</button>
   <button id="tUndo">Son noktayı sil</button><button id="tClear">Temizle</button>
  </div>
  <label>Görev adı<input id="ad" value="gorev"></label>
  <div class="row">
   <label>Hedef<select id="hedef"><option value="araba">Araba</option><option value="insan">İnsan (arama-kurtarma)</option><option value="ikisi">İkisi</option></select></label>
   <label>Araba modu<select id="mod"><option value="roof">Tavana</option><option value="dropzone">Teslimat noktasına</option></select></label>
  </div>
  <div class="row">
   <label>İrtifa (m)<select id="irtifa"><option>25</option><option>50</option><option>75</option></select></label>
   <label>Paket sayısı<input id="teslimat" type="number" min="1" max="10" value="3"></label>
  </div>
  <label>Rüzgar telafisi<select id="ruzgar"><option value="yok">Kapalı</option><option value="egim">Açık (drone eğiminden)</option></select></label>
  <button class="primary" id="save">Görevi kaydet (gorev.json)</button>
  <div class="summary" id="sum"></div>
 </div>
</main>
<script>
const META = __META__;
const img = new Image(); img.src = "data:image/jpeg;base64,__IMG__";
const c = document.getElementById("c"), ctx = c.getContext("2d");
const W = META.genislik_px, Hp = META.yukseklik_px, f = W / 2 / Math.tan(META.hfov_deg * Math.PI / 360);
const mpp = META.yukseklik_m / f;                       // metre / piksel
let mode = "area", area = null, drag = null, pts = [];
const toNE = (px, py) => [META.merkez_n - (py - Hp / 2) * mpp, META.merkez_e + (px - W / 2) * mpp];
const toPx = (n, e) => [W / 2 + (e - META.merkez_e) / mpp, Hp / 2 - (n - META.merkez_n) / mpp];
function evPx(ev) { const r = c.getBoundingClientRect(); return [(ev.clientX - r.left) * W / r.width, (ev.clientY - r.top) * Hp / r.height]; }
function draw() {
  c.width = W; c.height = Hp; ctx.drawImage(img, 0, 0);
  const s = W / 1000;
  // harita alanı (geofence sınırı için referans)
  const fa = META.harita_alani, [x0, y0] = toPx(fa[1], fa[2]), [x1, y1] = toPx(fa[0], fa[3]);
  ctx.setLineDash([10 * s, 8 * s]); ctx.strokeStyle = "rgba(255,255,255,.7)"; ctx.lineWidth = 2 * s; ctx.strokeRect(x0, y0, x1 - x0, y1 - y0); ctx.setLineDash([]);
  // iniş noktası
  const [hx, hy] = toPx(META.inis[0], META.inis[1]);
  ctx.fillStyle = "#1f8a4c"; ctx.beginPath(); ctx.arc(hx, hy, 9 * s, 0, 7); ctx.fill();
  ctx.font = `${16 * s}px sans-serif`; ctx.fillStyle = "#fff"; ctx.fillText("İniş", hx + 12 * s, hy + 5 * s);
  const a = drag || area;
  if (a) {
    const [ax0, ay0] = toPx(a[1], a[2]), [ax1, ay1] = toPx(a[0], a[3]);
    ctx.fillStyle = "rgba(212,56,44,.18)"; ctx.fillRect(ax0, ay0, ax1 - ax0, ay1 - ay0);
    ctx.strokeStyle = "#ff4a3a"; ctx.lineWidth = 3 * s; ctx.strokeRect(ax0, ay0, ax1 - ax0, ay1 - ay0);
  }
  pts.forEach((p, i) => {
    const [x, y] = toPx(p[0], p[1]);
    if (i > 0) { const [px, py] = toPx(pts[i - 1][0], pts[i - 1][1]); ctx.strokeStyle = "#ffd23f"; ctx.lineWidth = 2 * s; ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(x, y); ctx.stroke(); }
    ctx.fillStyle = "#ffd23f"; ctx.beginPath(); ctx.arc(x, y, 10 * s, 0, 7); ctx.fill();
    ctx.fillStyle = "#000"; ctx.font = `bold ${14 * s}px sans-serif`; ctx.fillText(String(i + 1), x - 4 * s, y + 5 * s);
  });
  summary();
}
function rectFrom(p, q) { return [Math.min(p[0], q[0]), Math.max(p[0], q[0]), Math.min(p[1], q[1]), Math.max(p[1], q[1])].map(v => Math.round(v * 10) / 10); }
c.addEventListener("mousedown", ev => { if (mode !== "area") return; const p = toNE(...evPx(ev)); drag = rectFrom(p, p); c._start = p; });
c.addEventListener("mousemove", ev => {
  const [n, e] = toNE(...evPx(ev)); document.getElementById("pos").textContent = `kuzey ${n.toFixed(1)} m, doğu ${e.toFixed(1)} m`;
  if (drag) { drag = rectFrom(c._start, [n, e]); draw(); }
});
window.addEventListener("mouseup", () => { if (drag) { if (drag[1] - drag[0] > 5 && drag[3] - drag[2] > 5) area = drag; drag = null; draw(); } });
c.addEventListener("click", ev => { if (mode !== "pt") return; const [n, e] = toNE(...evPx(ev)); pts.push([Math.round(n * 10) / 10, Math.round(e * 10) / 10]); draw(); });
const setMode = m => { mode = m; tArea.classList.toggle("on", m === "area"); tPt.classList.toggle("on", m === "pt"); };
tArea.onclick = () => setMode("area"); tPt.onclick = () => setMode("pt");
tUndo.onclick = () => { pts.pop(); draw(); }; tClear.onclick = () => { area = null; pts = []; draw(); };
["hedef", "mod", "irtifa", "teslimat", "ruzgar", "ad"].forEach(id => document.getElementById(id).addEventListener("input", () => { mod.disabled = hedef.value !== "araba"; summary(); }));
function lanes(a, alt) { const sp = 35 * Math.max(alt, 25) / 25; return Math.max(1, Math.ceil((a[3] - a[2]) / sp - 1e-9)); }
function summary() {
  const el = document.getElementById("sum"), alt = +irtifa.value;
  if (!area) { el.innerHTML = "Arama alanı yok: fareyle haritada bir dikdörtgen çiz."; return; }
  const n = lanes(area, alt), len = n * (area[1] - area[0]) + pts.length * 40;
  const fa = META.harita_alani, out = area[0] < fa[0] - 30 || area[1] > fa[1] + 30 || area[2] < fa[2] - 30 || area[3] > fa[3] + 30;
  el.innerHTML = `Alan: kuzey ${area[0]}…${area[1]} m, doğu ${area[2]}…${area[3]} m (${(area[1] - area[0]).toFixed(0)} × ${(area[3] - area[2]).toFixed(0)} m)\n`
    + `${pts.length} kontrol noktası, ${n} tarama şeridi (${alt} m)\n`
    + `Hedef: ${hedef.options[hedef.selectedIndex].text}${hedef.value === "araba" ? " — " + mod.options[mod.selectedIndex].text : ""}, ${teslimat.value} paket\n`
    + `Tahmini arama yolu ~${Math.round(len)} m`
    + (out ? `\n<span class="warn">Uyarı: alan haritanın dışına taşıyor; görev başlamaz.</span>` : "")
    + (hedef.value !== "araba" ? `\nKişilere paket en az 5 m uzağa bırakılır.` : "");
}
save.onclick = () => {
  if (!area) { alert("Önce arama alanını çiz."); return; }
  const plan = { ad: ad.value || "gorev", alan: area, noktalar: pts, hedef: hedef.value,
                 mod: hedef.value === "araba" ? mod.value : null, irtifa: +irtifa.value, teslimat: +teslimat.value,
                 ruzgar_telafi: ruzgar.value, olusturma: new Date().toISOString() };
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([JSON.stringify(plan, null, 2)], { type: "application/json" }));
  a.download = (plan.ad.replace(/[^\w\-]+/g, "_") || "gorev") + ".json"; a.click();
};
img.onload = draw;
</script></body></html>
"""


def main():
    meta = json.loads((MAP / "harita.json").read_text(encoding="utf-8"))
    b64 = base64.b64encode((MAP / "harita.jpg").read_bytes()).decode("ascii")
    out = MAP / "gorev_planlayici.html"
    out.write_text(PAGE.replace("__META__", json.dumps(meta)).replace("__IMG__", b64), encoding="utf-8")
    print(f"planlayıcı: {out} ({out.stat().st_size / 1e6:.1f} MB) — çift tıklayıp tarayıcıda aç")


if __name__ == "__main__":
    main()
