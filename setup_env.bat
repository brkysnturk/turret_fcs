@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
cd /d "%~dp0"

echo ==============================================================
echo    TURRET FCS // OTOMATIK ORTAM KURULUMU (SETUP ENV)
echo ==============================================================
echo.

rem ------------------------------------------------------------------
rem 1) Uygun Python bul: py -3.12/3.11/3.10, sonra PATH'teki python.
rem ------------------------------------------------------------------
set "PY_CMD="

if not defined PY_CMD (
    py -3.12 -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PY_CMD=py -3.12"
)
if not defined PY_CMD (
    py -3.11 -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PY_CMD=py -3.11"
)
if not defined PY_CMD (
    py -3.10 -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PY_CMD=py -3.10"
)
if not defined PY_CMD (
    python -c "import sys; raise SystemExit(0 if (3,10) <= sys.version_info[:2] <= (3,12) else 1)" >nul 2>nul
    if not errorlevel 1 set "PY_CMD=python"
)

if not defined PY_CMD (
    echo [HATA] Python 3.10 - 3.12 bulunamadi.
    echo        https://www.python.org/downloads/ adresinden kurun ve
    echo        kurulumda "Add python.exe to PATH" secenegini isaretleyin.
    if not defined CI pause
    exit /b 1
)

%PY_CMD% -c "import sys; print('[OK] Python %d.%d.%d (%s)' %% (sys.version_info[0], sys.version_info[1], sys.version_info[2], sys.executable))"

rem ------------------------------------------------------------------
rem 2) Sanal ortam: yoksa veya baska bir Python'a bagliysa yeniden kur.
rem ------------------------------------------------------------------
set "NEED_VENV=1"
if exist "venv\Scripts\python.exe" (
    venv\Scripts\python.exe -c "import sys; raise SystemExit(0 if (3,10) <= sys.version_info[:2] <= (3,12) else 1)" >nul 2>nul
    if not errorlevel 1 (
        set "NEED_VENV=0"
        echo [INFO] Mevcut venv kullanilabilir.
    ) else (
        echo [WARN] Mevcut venv bozuk veya farkli Python surumune bagli; yeniden kuruluyor.
    )
)

if "!NEED_VENV!"=="1" (
    echo [1/4] Sanal ortam olusturuluyor (venv)...
    if exist "venv" rmdir /s /q "venv"
    %PY_CMD% -m venv venv
    if errorlevel 1 (
        echo [HATA] venv olusturulamadi.
        if not defined CI pause
        exit /b 1
    )
    echo [OK] venv olusturuldu.
) else (
    echo [1/4] Sanal ortam hazir.
)

rem ------------------------------------------------------------------
rem 3) pip ve temel araclar
rem ------------------------------------------------------------------
echo.
echo [2/4] pip guncelleniyor...
venv\Scripts\python.exe -m pip install --upgrade pip
if errorlevel 1 (
    echo [HATA] pip guncellenemedi. Internet baglantisini kontrol edin.
    if not defined CI pause
    exit /b 1
)

rem ------------------------------------------------------------------
rem 4) Bagimliliklar
rem    PyTorch CPU wheel'i ayri indeksten kurulur; boylece 2.5 GB'lik CUDA
rem    indirmesi yapilmaz ve surum pinsiz torch en kucuk paketle gelir.
rem ------------------------------------------------------------------
echo.
echo [3/4] Bagimliliklar yukleniyor (requirements.txt)...
venv\Scripts\python.exe -m pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cpu
if errorlevel 1 (
    echo [HATA] Bagimliliklar kurulamadi.
    if not defined CI pause
    exit /b 1
)

echo.
echo [4/4] Paket gelistirme modunda kuruluyor...
venv\Scripts\python.exe -m pip install --no-deps -e .
if errorlevel 1 (
    echo [HATA] Paket kurulumu basarisiz.
    if not defined CI pause
    exit /b 1
)

echo.
echo [Dogrulama] Temel importlar kontrol ediliyor...
venv\Scripts\python.exe -c "import numpy, cv2, torch, ultralytics, serial, turret_fcs; print('[OK] numpy %%s, opencv %%s, torch %%s' %% (numpy.__version__, cv2.__version__, torch.__version__))"
if errorlevel 1 (
    echo [HATA] Kurulum dogrulanamadi.
    if not defined CI pause
    exit /b 1
)

echo.
echo ==============================================================
echo  [TAMAMLANDI] Tum kutuphaneler basariyla kuruldu!
echo  Simdi 'main.bat' ile sistemi calistirabilirsiniz.
echo  Testler icin: venv\Scripts\python.exe -m pytest -q
echo ==============================================================
if not defined CI pause
endlocal
