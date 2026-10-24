"""
================================================================================
MODUL: MODEL AGIRLIKLARI YONETICISI (WEIGHTS MANAGER)
================================================================================
Bu modul, YOLOv8 balon tespit modeli (best.pt) ve temel nesne modeli (yolov8n.pt)
agirliklarinin varligini kontrol eder. Agirliklar yerel diskte bulunmadiginda,
GitHub Releases varliklarindan (Release Assets) guvenli ve otomatik sekilde indirir.

Depo boyutunu kucuk tutmak ve Git gecmisini sisirmemek icin .pt dosyalari Git LFS
veya GitHub Releases uzerinden dagitilir.
================================================================================
"""

import sys
import urllib.request
from pathlib import Path
from typing import Optional

REPO_OWNER = "brkysnturk"
REPO_NAME = "turret_fcs"
DEFAULT_RELEASE_TAG = "v1.4"
RELEASE_URL_TEMPLATE = "https://github.com/{owner}/{repo}/releases/download/{tag}/{filename}"

KNOWN_WEIGHTS = {
    "best.pt": "Ozel egitilmis hava hedefi (balon) tespit modeli",
    "yolov8n.pt": "Ultralytics YOLOv8 Nano temel model",
}


def download_weight_file(filename: str, dest_path: Path, tag: str = DEFAULT_RELEASE_TAG) -> bool:
    """GitHub Releases asset URL'sinden belirtilen agirlik dosyasini indirir."""
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    url = RELEASE_URL_TEMPLATE.format(
        owner=REPO_OWNER,
        repo=REPO_NAME,
        tag=tag,
        filename=filename,
    )
    temp_path = dest_path.with_suffix(".tmp")
    print(f"[WEIGHTS] Agirlik dosyasi bulunamadi: {filename}")
    print(f"[WEIGHTS] GitHub Releases'den indiriliyor: {url}")
    try:
        def _report(count, block_size, total_size):
            if total_size > 0:
                percent = int(count * block_size * 100 / total_size)
                mb_downloaded = (count * block_size) / (1024 * 1024)
                mb_total = total_size / (1024 * 1024)
                sys.stdout.write(f"\r[WEIGHTS] Indiriliyor: %{min(100, percent)} ({mb_downloaded:.1f}/{mb_total:.1f} MB)")
                sys.stdout.flush()

        urllib.request.urlretrieve(url, temp_path, reporthook=_report)
        sys.stdout.write("\n")
        temp_path.replace(dest_path)
        print(f"[WEIGHTS] Basariyla kaydedildi: {dest_path}")
        return True
    except Exception as e:
        if temp_path.exists():
            temp_path.unlink()
        print(f"\n[WEIGHTS HATA] Otomatik indirme basarisiz oldu: {e}")
        print(f"[WEIGHTS BILGI] Dosyayi elle indirmek icin: {url}")
        print(f"[WEIGHTS BILGI] Hedef dizin: {dest_path}")
        return False


def ensure_weight_file(model_path: str | Path, tag: str = DEFAULT_RELEASE_TAG) -> Path:
    """
    Model dosyasinin varligini kontrol eder, yoksa GitHub Releases'den indirmeyi dener.
    Dosya bulunamaz veya indirilemezse ayni yolu dondurur (cagiran taraf istisnayi yonetir).
    """
    path = Path(model_path)
    if path.exists() and path.stat().st_size > 1000:
        return path

    filename = path.name
    if filename in KNOWN_WEIGHTS:
        download_weight_file(filename, path, tag=tag)
    return path


def ensure_all_weights(weights_dir: Optional[Path] = None, tag: str = DEFAULT_RELEASE_TAG) -> None:
    """Tum bilinen agirliklarin weights/ altinda hazir olmasini saglar."""
    if weights_dir is None:
        weights_dir = Path(__file__).resolve().parents[1] / "weights"
    weights_dir.mkdir(parents=True, exist_ok=True)
    for filename in KNOWN_WEIGHTS:
        target = weights_dir / filename
        if not target.exists() or target.stat().st_size < 1000:
            download_weight_file(filename, target, tag=tag)
        else:
            size_mb = target.stat().st_size / (1024 * 1024)
            print(f"[WEIGHTS] Mevcut: {filename} ({size_mb:.2f} MB)")


if __name__ == "__main__":
    ensure_all_weights()
