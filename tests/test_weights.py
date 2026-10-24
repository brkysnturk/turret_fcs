"""
Test suite for turret_fcs.weights manager: SHA256 verification and path resolution.
"""

from pathlib import Path
import pytest

from turret_fcs.weights import (
    KNOWN_WEIGHTS,
    compute_file_sha256,
    ensure_weight_file,
)


def test_known_weights_metadata():
    """KNOWN_WEIGHTS must contain best.pt and yolov8n.pt with valid 64-char hex SHA256 hashes."""
    assert "best.pt" in KNOWN_WEIGHTS
    assert "yolov8n.pt" in KNOWN_WEIGHTS

    for name, info in KNOWN_WEIGHTS.items():
        assert "sha256" in info
        assert len(info["sha256"]) == 64
        int(info["sha256"], 16)  # must be valid hex


def test_compute_sha256_of_temp_file(tmp_path: Path):
    """compute_file_sha256 must correctly hash arbitrary content."""
    sample_file = tmp_path / "sample.bin"
    sample_file.write_bytes(b"Turret FCS SHA256 Test")
    
    # Precomputed SHA256 for b"Turret FCS SHA256 Test"
    import hashlib
    expected = hashlib.sha256(b"Turret FCS SHA256 Test").hexdigest()
    
    actual = compute_file_sha256(sample_file)
    assert actual == expected


def test_ensure_weight_file_existing(tmp_path: Path):
    """ensure_weight_file should immediately return path if file exists with content."""
    dummy_weight = tmp_path / "custom_model.pt"
    dummy_weight.write_bytes(b"0" * 2000)
    
    resolved = ensure_weight_file(dummy_weight, required=True)
    assert resolved == dummy_weight


def test_ensure_weight_file_missing_raises(tmp_path: Path):
    """ensure_weight_file must raise FileNotFoundError if file is missing and cannot be downloaded."""
    non_existent = tmp_path / "unknown_model_12345.pt"
    with pytest.raises(FileNotFoundError):
        ensure_weight_file(non_existent, required=True)


def test_download_falls_back_to_legacy_tag(tmp_path: Path, monkeypatch):
    """A release tag without assets must fall back to the last published weights tag."""
    import turret_fcs.weights as weights

    attempts = []

    def fake_download(filename, dest_path, tag, timeout, verify_checksum):
        attempts.append(tag)
        if tag == weights.LEGACY_RELEASE_TAG:
            Path(dest_path).write_bytes(b"x" * 2048)
            return True
        return False

    monkeypatch.setattr(weights, "download_weight_file_from_tag", fake_download)

    target = tmp_path / "best.pt"
    ok = weights.download_weight_file("best.pt", target, tag="v2.0")

    assert ok is True
    assert attempts == ["v2.0", weights.LEGACY_RELEASE_TAG]
    assert target.exists()
