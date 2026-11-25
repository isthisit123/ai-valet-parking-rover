"""편집한 UNO Q 앱 소스 네 파일을 App Lab 가져오기 ZIP으로 다시 묶는다."""
from datetime import datetime, timezone
from pathlib import Path
from tempfile import NamedTemporaryFile
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parent
APP_NAME = 'q_rover_encoder_receiver'
FILES = ('app.yaml', 'python/main.py', 'sketch/sketch.ino', 'sketch/sketch.yaml')


def package(root=ROOT):
    root = Path(root)
    sources = {name: (root / APP_NAME / name).read_bytes() for name in FILES}
    output = root / f'{APP_NAME}.zip'
    with NamedTemporaryFile(dir=root, suffix='.zip', delete=False) as file:
        temporary = Path(file.name)
    try:
        with ZipFile(temporary, 'w', ZIP_DEFLATED) as archive:
            for name, data in sources.items():
                archive.writestr(f'{APP_NAME}/{name}', data)
        if output.exists():
            backups = root / 'backups'
            backups.mkdir(exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            with (backups / f'receiver_before_{stamp}.zip').open('xb') as file:
                file.write(output.read_bytes())
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    return output


if __name__ == '__main__':
    print(package())
