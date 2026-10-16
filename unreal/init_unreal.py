# Editör her açılışta bunu çalıştırır. Sadece gece otomasyonu sırasında (C:/UE/flags/auto varken)
# auto_editor.py'yi devreye sokar; normal kullanımda hiçbir şey yapmaz.
from pathlib import Path

if Path("C:/UE/flags/auto").exists():
    import runpy
    runpy.run_path("C:/UE/auto_editor.py", run_name="__main__")
