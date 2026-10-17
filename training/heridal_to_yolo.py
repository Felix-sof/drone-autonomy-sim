"""HERIDAL (arama-kurtarma, gerçek drone görüntüleri, tek sınıf "person") -> YOLO, iki sınıflı şema.

Kaynak: Zenodo 5662351 (keras-retinanet PASCAL VOC sürümü, CC-BY-3.0). Görüntüler 4000x3000,
kişiler ~60 px. Görüntü 2x2 parçaya bölünür; 1280'de eğitilince kişi ~38 px olur (drone'umuz 25 m'de
kişiyi 40-90 px görüyor). Kişisiz parçaların sadece NEG_KEEP oranı tutulur.

  python training/heridal_to_yolo.py C:/UE/heridal_dl/heridal C:/UE/heridal/yolo
Sınıf numarası: person = 1 (car = 0 bizim şemada; HERIDAL'da araba etiketi yok).
"""
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2

PERSON = 1
TILES = 2          # 2x2 parça
NEG_KEEP = 0.25    # kişisiz parçaların tutulma oranı
MIN_PX = 6         # parça kenarında kırpılıp bu kadar küçülen kutular atılır


def boxes_from_xml(xml):
    out = []
    for o in ET.parse(xml).getroot().iter("object"):
        b = o.find("bndbox")
        out.append(tuple(float(b.find(k).text) for k in ("xmin", "ymin", "xmax", "ymax")))
    return out


def find_image(xml, roots):
    stem = xml.stem
    for r in roots:
        for ext in (".JPG", ".jpg", ".png", ".jpeg"):
            p = r / f"{stem}{ext}"
            if p.exists():
                return p
    return None


def main(src, dst):
    src, dst = Path(src), Path(dst)
    rng = random.Random(0)
    xmls = sorted(src.rglob("*.xml"))
    print(f"{len(xmls)} etiket dosyası")
    n = {"train": 0, "val": 0}
    nbox = 0
    for xml in xmls:
        # resmi ayrım dosya adında: train_*.xml / test_*.xml
        split = "val" if xml.stem.lower().startswith("test") else "train"
        img_path = find_image(xml, [xml.parent, xml.parent.parent / "images", xml.parent.parent / "JPEGImages"])
        if img_path is None:
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        H, W = img.shape[:2]
        boxes = boxes_from_xml(xml)
        th, tw = H // TILES, W // TILES
        for ty in range(TILES):
            for tx in range(TILES):
                x0, y0 = tx * tw, ty * th
                lines = []
                for x1, y1, x2, y2 in boxes:
                    cx1, cy1 = max(x1, x0) - x0, max(y1, y0) - y0
                    cx2, cy2 = min(x2, x0 + tw) - x0, min(y2, y0 + th) - y0
                    if cx2 - cx1 < MIN_PX or cy2 - cy1 < MIN_PX:
                        continue
                    lines.append(f"{PERSON} {(cx1 + cx2) / 2 / tw:.6f} {(cy1 + cy2) / 2 / th:.6f} "
                                 f"{(cx2 - cx1) / tw:.6f} {(cy2 - cy1) / th:.6f}")
                if not lines and rng.random() > NEG_KEEP:
                    continue
                name = f"heridal_{xml.stem}_{ty}{tx}"
                for sub in ("images", "labels"):
                    (dst / sub / split).mkdir(parents=True, exist_ok=True)
                tile = img[y0:y0 + th, x0:x0 + tw]
                (dst / "images" / split / f"{name}.jpg").write_bytes(cv2.imencode(".jpg", tile)[1].tobytes())
                (dst / "labels" / split / f"{name}.txt").write_text("\n".join(lines))
                n[split] += 1
                nbox += len(lines)
    (dst / "data.yaml").write_text(f"path: {dst.as_posix()}\ntrain: images/train\nval: images/val\n"
                                   f"names:\n  0: car\n  1: person\n", encoding="utf-8")
    print(f"parça: train {n['train']}, val {n['val']}, kişi kutusu {nbox} -> {dst}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
