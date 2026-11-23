"""Read receiver source from the delivery ZIP or a locally edited app folder."""
from pathlib import Path
from zipfile import ZipFile

APP_NAME = "q_rover_encoder_receiver"
ROOT = Path(__file__).resolve().parents[1]


def read_receiver(relative_path):
    source = ROOT / APP_NAME / relative_path
    if source.is_file():
        return source.read_text(encoding="utf-8")
    with ZipFile(ROOT / f"{APP_NAME}.zip") as archive:
        return archive.read(f"{APP_NAME}/{relative_path}").decode("utf-8")
