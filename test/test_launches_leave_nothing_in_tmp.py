"""A launch's parameter file lands in this session's own TMPDIR, not in /tmp.

launch_ros writes every Node's parameter dict to
NamedTemporaryFile(prefix='launch_params_', delete=False) and never removes it
(BUGS B-142: 784,505 of them filled a 7.8 GB RAM-disk /tmp). The root conftest
gives the session a TMPDIR that is deleted when the session ends.
"""
import os
import subprocess
import sys
import tempfile

# Read from the environment, not `import conftest`: under a multi-package run
# another directory's conftest.py can win that import.
SESSION_TMP = os.environ.get('TMPDIR', '')


def test_temporary_files_go_to_the_session_directory():
    f = tempfile.NamedTemporaryFile(prefix='launch_params_', delete=False)
    f.close()
    assert os.path.dirname(f.name) == SESSION_TMP
    assert os.path.basename(SESSION_TMP).startswith('mongla-test-session-')


def test_a_launched_process_inherits_it():
    out = subprocess.run([sys.executable, '-c', 'import tempfile; print(tempfile.gettempdir())'],
                         capture_output=True, text=True, check=True).stdout.strip()
    assert out == SESSION_TMP


def test_pytests_own_temp_root_did_not_move(tmp_path):
    assert SESSION_TMP not in str(tmp_path)
