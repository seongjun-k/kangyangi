'''
Vosk 기반 한국어 오프라인 음성 인식 모듈. web_operate.py의 /voice 엔드포인트에서 사용.

vosk 패키지 또는 모델이 없어도 이 모듈의 import 자체는 항상 성공한다(임포트 가드) —
web_operate.py는 음성 기능만 비활성화된 채로 정상 기동해야 하므로, 실패는
recognize()가 빈 문자열을 반환하는 형태로만 드러난다.
'''

import json
import sys
import tempfile
import threading
import urllib.request
import zipfile
from pathlib import Path

try:
    import vosk
    _VOSK_AVAILABLE = True
except ImportError:
    _VOSK_AVAILABLE = False

MODEL_URL = "https://alphacephei.com/vosk/models/vosk-model-small-ko-0.22.zip"
MODELS_DIR = Path(__file__).parent / "models"
MODEL_DIR = MODELS_DIR / "vosk-model-small-ko-0.22"
SAMPLE_RATE = 16000

_model = None
_model_lock = threading.Lock()


def available():
    return _VOSK_AVAILABLE


def _ensure_model():
    '''모델 디렉터리가 없으면 최초 1회 다운로드+압축 해제.
    임시 디렉터리에 풀고 완료 후 최종 경로로 rename(원자적) — 다운로드 중단 시
    불완전한 MODEL_DIR이 남아 "이미 있음"으로 오판되는 것을 방지.'''
    if MODEL_DIR.exists():
        return True
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    zip_path = MODELS_DIR / "vosk-model-small-ko-0.22.zip"
    try:
        urllib.request.urlretrieve(MODEL_URL, zip_path)
        with tempfile.TemporaryDirectory(dir=MODELS_DIR) as tmp_dir:
            with zipfile.ZipFile(zip_path) as zf:
                zf.extractall(tmp_dir)
            extracted = Path(tmp_dir) / MODEL_DIR.name
            extracted.rename(MODEL_DIR)
    finally:
        if zip_path.exists():
            zip_path.unlink()
    return MODEL_DIR.exists()


def _get_recognizer():
    global _model
    if _model is None:
        if not _ensure_model():
            raise RuntimeError("vosk model download failed")
        vosk.SetLogLevel(-1)  # 콘솔 소음 억제
        _model = vosk.Model(str(MODEL_DIR))
    return vosk.KaldiRecognizer(_model, SAMPLE_RATE)


def preload_model_async():
    '''서버 기동 시 백그라운드 스레드로 모델을 미리 로드해 첫 PTT 블로킹을 없앤다.'''
    if not _VOSK_AVAILABLE:
        return
    def _run():
        try:
            with _model_lock:
                _get_recognizer()
        except Exception as e:
            print(f"[voice] 모델 선로딩 실패: {e}", file=sys.stderr)
    threading.Thread(target=_run, daemon=True).start()


def recognize(pcm_bytes):
    '''PCM bytes(16kHz s16le mono) -> 인식 텍스트. 미설치/모델없음/오류 시 빈 문자열.'''
    if not _VOSK_AVAILABLE or not pcm_bytes:
        return ""
    try:
        with _model_lock:
            rec = _get_recognizer()
            rec.AcceptWaveform(bytes(pcm_bytes))
            result = json.loads(rec.FinalResult())
        return result.get("text", "")
    except Exception as e:
        print(f"[voice] 인식 실패: {e}", file=sys.stderr)
        return ""
