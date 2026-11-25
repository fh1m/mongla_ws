"""An override the loader cannot use is reported, never dropped or raised (audit vl_14 F1, F2)."""
from mongla_vision import target_geometry as tg


def _with(tmp_path, monkeypatch, text):
    f = tmp_path / 'target_geometry.yaml'
    f.write_text(text)
    monkeypatch.setenv('MONGLA_TARGET_GEOMETRY', str(f))
    monkeypatch.setattr(tg, '_override_paths', lambda: [str(f)])
    tg._CACHE.clear()
    tg._STAMPS.clear()
    return str(f)


def test_a_scalar_override_is_reported(tmp_path, monkeypatch):
    _with(tmp_path, monkeypatch, 'red_pipe: 0.0334\n')
    assert any(r[0] == 'red_pipe' for r in tg.rejected_overrides())


def test_a_misspelt_width_key_is_reported(tmp_path, monkeypatch):
    _with(tmp_path, monkeypatch, 'red_pipe: {width: 0.0334}\n')
    assert any(r[0] == 'red_pipe.width' for r in tg.rejected_overrides())


def test_a_list_file_is_an_error_not_a_crash(tmp_path, monkeypatch):
    path = _with(tmp_path, monkeypatch, '- red_pipe\n')
    assert any(e[0] == path for e in tg.load_errors())
    assert tg.width_for('gate') > 0.0
