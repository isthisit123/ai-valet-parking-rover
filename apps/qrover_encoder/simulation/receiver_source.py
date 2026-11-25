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
        # App Lab Export는 앱 폴더 이름에 하이픈을 사용할 수 있다.
        matches = [name for name in archive.namelist() if name.endswith('/' + relative_path)]
        if len(matches) != 1:
            raise ValueError(f'수신 앱 ZIP에서 유일한 {relative_path} 파일이 필요합니다.')
        return archive.read(matches[0]).decode("utf-8")
